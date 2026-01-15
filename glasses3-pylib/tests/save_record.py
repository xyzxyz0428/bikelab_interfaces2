"""
save_recording_with_stop_on_ctrlc.py

Connects to Glasses3, waits for (or enables) NTP sync, starts recording, runs until you press Ctrl+C,
stops recording, collects metadata and attempts to download any HTTP-accessible recording files,
and saves everything under a timestamped folder.

Revision: If time synchronization fails within the configured timeout, the script will set the
device time to the current local Linux system time using g3.system.set_time(...) and then continue
to start recording. The rest of the script behavior remains the same.

Usage:
  - Set environment variable G3_HOSTNAME to the glasses hostname or IP (or pass it in code).
  - Optional: set OUTPUT_DIR to choose where recording folders are created (defaults to ./recordings)
  - Run: python save_recording_with_stop_on_ctrlc.py

Dependencies:
  pip install glasses3-pylib aiohttp python-dotenv
"""
import asyncio
import json
import logging
import os
import signal
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import Optional, Tuple, Any, Dict, List

import aiohttp
from dotenv import load_dotenv

from g3pylib import connect_to_glasses

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("g3-save")

# Configuration
DEFAULT_OUTPUT_BASE = Path(os.environ.get("OUTPUT_DIR", "./recordings"))
G3_HOSTNAME = os.environ.get("G3_HOSTNAME")  # required, unless you change the code to use zeroconf
NTP_WAIT_TIMEOUT = float(os.environ.get("NTP_WAIT_TIMEOUT", "10"))  # seconds
CLOCK_OFFSET_THRESHOLD_SECONDS = float(os.environ.get("CLOCK_OFFSET_THRESHOLD_SECONDS", "0.5"))
FINALIZE_WAIT_SECONDS = float(os.environ.get("FINALIZE_WAIT_SECONDS", "3.0"))  # wait for device to finalize files


async def wait_for_ntp_sync(g3, timeout_s: float = NTP_WAIT_TIMEOUT, poll_interval: float = 1.0) -> bool:
    """Poll until device reports NTP synchronization or until timeout."""
    start = asyncio.get_event_loop().time()
    while True:
        try:
            sync = await g3.system.get_ntp_is_synchronized()
        except Exception as e:
            logger.warning("Failed to read ntp-is-synchronized: %s", e)
            sync = False
        logger.info("ntp-is-synchronized = %s", sync)
        if sync:
            return True
        if asyncio.get_event_loop().time() - start > timeout_s:
            return False
        await asyncio.sleep(poll_interval)


async def check_clock_offset(g3) -> timedelta:
    """Return device_time - local_utc_time as timedelta (positive if device ahead)."""
    device_time = await g3.system.get_time()
    # treat naive device_time as UTC
    if device_time.tzinfo is None:
        device_time = device_time.replace(tzinfo=timezone.utc)
    local_utc = datetime.now(timezone.utc)
    offset = device_time - local_utc
    logger.info("device_time=%s local_utc=%s offset=%s", device_time.isoformat(), local_utc.isoformat(), offset)
    return offset


async def download_url(session: aiohttp.ClientSession, url: str, dest: Path) -> Tuple[bool, Optional[str]]:
    """Download a single URL and save it to dest. Returns (ok, filename_or_error)."""
    try:
        async with session.get(url) as resp:
            if resp.status != 200:
                # try read short body for diagnostics
                try:
                    text = await resp.text()
                except Exception:
                    text = "<no text body>"
                return False, f"HTTP {resp.status}: {text[:200]}"

            cd = resp.headers.get("Content-Disposition")
            if cd and "filename=" in cd:
                filename = cd.split("filename=")[-1].strip(' "')
            else:
                filename = url.rstrip("/").split("/")[-1] or "download"

            if dest.is_dir():
                file_path = dest / filename
            else:
                file_path = dest

            counter = 1
            base = file_path.stem
            suffix = file_path.suffix
            while file_path.exists():
                file_path = file_path.with_name(f"{base}_{counter}{suffix}")
                counter += 1

            with file_path.open("wb") as fh:
                async for chunk in resp.content.iter_chunked(65536):
                    fh.write(chunk)

            return True, str(file_path)

    except Exception as e:
        return False, str(e)



async def try_download_recording_files(http_base: str, http_path: str, out_dir: Path):
    """
    Best-effort attempt to download files referenced by the recording http_path.

    Supports the common Tobii Glasses 3 pattern where GET /recordings/<uuid>
    returns a JSON metadata object that contains many nested {"file": "..."} fields
    (scenevideo.mp4, gazedata.gz, eventdata.gz, imudata.gz, snapshots, etc.).

    Strategy:
      1) GET the recording URL (http_base + http_path)
      2) If JSON, recursively extract all values of key "file"
      3) For each filename, try two candidate URLs:
            /recordings/<uuid>/<filename>
            /recordings/<uuid>/<meta-folder>/<filename>
         because some firmwares keep resources in a "meta" folder.
      4) Save the original JSON to out_dir/index.json for traceability.
    """
    def extract_files(obj):
        """Recursively extract all values of key 'file' from nested dict/list JSON."""
        files = []
        if isinstance(obj, dict):
            for k, v in obj.items():
                if k == "file" and isinstance(v, str):
                    files.append(v)
                else:
                    files.extend(extract_files(v))
        elif isinstance(obj, list):
            for item in obj:
                files.extend(extract_files(item))
        return files

    out_dir.mkdir(parents=True, exist_ok=True)

    async with aiohttp.ClientSession() as session:
        full_url = (http_base.rstrip("/") + "/" + http_path.lstrip("/"))
        logger.info("Attempting to GET recording http url: %s", full_url)

        try:
            async with session.get(full_url) as resp:
                ctype = resp.headers.get("Content-Type", "")
                body = await resp.read()

                # Always save the raw response for debugging
                (out_dir / "index.json").write_bytes(body)

                if resp.status != 200:
                    logger.warning("HTTP GET %s returned status %s. Saved body to index.json",
                                   full_url, resp.status)
                    return

                # Try parse JSON
                js = None
                try:
                    if ("application/json" in ctype) or body.strip().startswith((b"{", b"[")):
                        js = json.loads(body.decode("utf-8"))
                except Exception as e:
                    logger.warning("Response from %s is not valid JSON: %s", full_url, e)

                if js is None:
                    # Not JSON; save as a single blob file
                    name = http_path.rstrip("/").split("/")[-1] or "recording"
                    if "." not in name:
                        if "video" in ctype:
                            ext = ".mp4"
                        elif "audio" in ctype:
                            ext = ".wav"
                        elif "json" in ctype:
                            ext = ".json"
                        else:
                            ext = ".bin"
                        name = f"{name}{ext}"
                    file_path = out_dir / name
                    file_path.write_bytes(body)
                    logger.info("Saved non-JSON response from %s to %s (content-type=%s)",
                                full_url, file_path, ctype)
                    return

                # JSON parsed: extract filenames
                if isinstance(js, dict):
                    uuid = js.get("uuid") or http_path.rstrip("/").split("/")[-1]
                    meta_folder = js.get("meta-folder", "meta")
                    files = extract_files(js)
                else:
                    # If the endpoint returns a list, still try extracting "file" recursively
                    uuid = http_path.rstrip("/").split("/")[-1]
                    meta_folder = "meta"
                    files = extract_files(js)

                files = sorted(set(f for f in files if isinstance(f, str) and f.strip()))
                if not files:
                    logger.info("JSON response had no 'file' entries; kept index.json in %s", out_dir)
                    return

                logger.info("Found %d file entries in JSON: %s", len(files), files)

                # Download each referenced file
                base_rec_url = (http_base.rstrip("/") + "/" + http_path.lstrip("/")).rstrip("/")
                for fname in files:
                    fname = fname.lstrip("/")

                    candidates = [
                        f"{base_rec_url}/{fname}",
                        f"{base_rec_url}/files/{fname}",
                        f"{base_rec_url}/{meta_folder}/{fname}",
                        f"{base_rec_url}/{meta_folder}/files/{fname}",
                    ]

                    downloaded = False
                    for file_url in candidates:
                        ok, info = await download_url(session, file_url, out_dir)
                        if ok:
                            logger.info("Downloaded %s -> %s", file_url, info)
                            downloaded = True
                            break

                    if not downloaded:
                        logger.warning("Failed to download '%s': %s (tried: %s)", fname, info, candidates)


                logger.info("Done. Files saved under %s", out_dir)

        except Exception as e:
            logger.exception("HTTP error while getting %s: %s", full_url, e)



async def save_recording_metadata(recording, out_dir: Path) -> Dict[str, Any]:
    """Collect metadata from recording object and save it as metadata.json in out_dir."""
    meta: Dict[str, Any] = {}
    try:
        meta["uuid"] = getattr(recording, "uuid", None)
        for prop in (
            "get_created",
            "get_duration",
            "get_folder",
            "get_gaze_overlay",
            "get_gaze_samples",
            "get_http_path",
            "get_name",
            "get_rtsp_path",
            "get_timezone",
            "get_valid_gaze_samples",
            "get_visible_name",
        ):
            if hasattr(recording, prop):
                try:
                    val = await getattr(recording, prop)()
                    if isinstance(val, datetime):
                        val = val.replace(tzinfo=timezone.utc).isoformat()
                    elif isinstance(val, timedelta):
                        val = val.total_seconds()
                    meta[prop] = val
                except Exception as e:
                    meta[prop] = f"ERROR: {e}"
    except Exception:
        logger.exception("Error collecting recording metadata")
    out_dir.mkdir(parents=True, exist_ok=True)
    meta_file = out_dir / "metadata.json"
    meta_file.write_text(json.dumps(meta, indent=2, default=str))
    logger.info("Saved recording metadata to %s", meta_file)
    return meta


async def start_and_run_recording(hostname: str, output_base: Path):
    """
    High-level flow (revised):
    - Connect
    - Ensure NTP sync or enable NTP
    - If NTP sync still not achieved within timeout, set device time to current local system time
      using g3.system.set_time(...)
    - Start recorder (subscribe to started signal)
    - Run until Ctrl+C (SIGINT)
    - Stop recorder and finalize
    - Find the latest recording, save metadata and try to download HTTP files
    """
    if not hostname:
        raise RuntimeError("Provide hostname (set G3_HOSTNAME env or call function with hostname).")
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session_folder = output_base / f"recording_{timestamp}"
    session_folder.mkdir(parents=True, exist_ok=True)
    logger.info("Output base folder: %s", session_folder.resolve())

    stop_event = asyncio.Event()

    # Register signal handler to stop on Ctrl+C
    loop = asyncio.get_running_loop()

    def _on_sigint():
        logger.info("SIGINT received, will stop recording...")
        stop_event.set()

    try:
        loop.add_signal_handler(signal.SIGINT, _on_sigint)
    except NotImplementedError:
        logger.debug("loop.add_signal_handler not implemented in this environment; KeyboardInterrupt will be used.")

    # Connect to glasses
    async with connect_to_glasses.with_hostname(hostname) as g3:
        logger.info("Connected to Glasses3 at %s", hostname)

        # Check NTP
        try:
            ntp_enabled = await g3.system.get_ntp_is_enabled()
            logger.info("ntp-is-enabled = %s", ntp_enabled)
        except Exception as e:
            logger.warning("Could not read ntp-is-enabled: %s", e)
            ntp_enabled = False
        try:
            ntp_sync = await g3.system.get_ntp_is_synchronized()
            logger.info("ntp-is-synchronized = %s", ntp_sync)
        except Exception as e:
            logger.warning("Could not read ntp-is-synchronized: %s", e)
            ntp_sync = False

        # Try to achieve NTP sync if possible, otherwise fall back to setting device time
        ntp_ok = ntp_sync
        if not ntp_ok and ntp_enabled:
            ntp_ok = await wait_for_ntp_sync(g3, timeout_s=NTP_WAIT_TIMEOUT)
            if not ntp_ok:
                logger.warning("NTP did not synchronize within timeout (%ss).", NTP_WAIT_TIMEOUT)
        elif not ntp_ok and not ntp_enabled:
            # try to enable NTP
            try:
                logger.info("Enabling NTP on device...")
                await g3.system.use_ntp(True)
                ntp_ok = await wait_for_ntp_sync(g3, timeout_s=NTP_WAIT_TIMEOUT)
                if not ntp_ok:
                    logger.warning("NTP enabled but did not synchronize within timeout (%ss).", NTP_WAIT_TIMEOUT)
            except Exception as e:
                logger.warning("Failed to enable NTP: %s", e)

        # If NTP still not synchronized, set the device time to current local Linux system time
        if not ntp_ok:
            try:
                # Use local system time expressed as UTC but send a naive datetime (the library appends 'Z')
                local_utc_naive = datetime.now(timezone.utc).replace(tzinfo=None)
                logger.info("Setting device time to local system UTC time: %s (naive UTC)", local_utc_naive.isoformat())
                set_ok = await g3.system.set_time(local_utc_naive)
                logger.info("g3.system.set_time(...) returned: %s", set_ok)
                # Optionally re-check device time and log offset
                try:
                    await asyncio.sleep(0.5)
                    offset_after = await check_clock_offset(g3)
                    if abs(offset_after.total_seconds()) > CLOCK_OFFSET_THRESHOLD_SECONDS:
                        logger.warning("Clock offset after set_time is %s (threshold %ss).", offset_after, CLOCK_OFFSET_THRESHOLD_SECONDS)
                except Exception:
                    logger.debug("Could not verify device time after set_time")
            except Exception as e:
                logger.exception("Failed to set device time from local system time: %s", e)

        # Proceed with starting the recording as before

        # Prepare to subscribe to started signal
        queue, unsubscribe = await g3.recorder.subscribe_to_started()
        logger.info("Subscribed to recorder 'started' signal")

        # Snapshot existing recording UUIDs so we can detect a new one later
        existing_uuids = set()
        try:
            async with g3.recordings.keep_updated_in_context():
                for r in g3.recordings.children:
                    existing_uuids.add(getattr(r, "uuid", None))
        except Exception:
            logger.debug("Could not read existing recordings before start.")

        # Start recording
        logger.info("Requesting recorder.start()")
        try:
            ok = await g3.recorder.start()
            logger.info("recorder.start() returned: %s", ok)
        except Exception as e:
            logger.exception("Failed to request recorder.start(): %s", e)
            # clean up
            try:
                await unsubscribe
            except Exception:
                pass
            return

        # Wait for recorder started signal (best-effort)
        try:
            sig = await asyncio.wait_for(queue.get(), timeout=5.0)
            logger.info("Received recorder started signal: %s", sig)
        except asyncio.TimeoutError:
            logger.warning("No 'started' signal received within 5s; continuing (the recorder may still be running).")

        # Now run until user presses Ctrl+C (stop_event)
        try:
            logger.info("Recording... press Ctrl+C to stop.")
            while not stop_event.is_set():
                await asyncio.sleep(0.5)
        except KeyboardInterrupt:
            logger.info("KeyboardInterrupt caught, will stop recording...")
            stop_event.set()
        finally:
            # Stop recording
            try:
                logger.info("Requesting recorder.stop()")
                await g3.recorder.stop()
            except Exception:
                logger.exception("Failed to call recorder.stop()")

            # Unsubscribe from started signal
            try:
                await unsubscribe
            except Exception:
                logger.debug("Failed to unsubscribe from started signal")

            # Give the device a moment to finalize files
            await asyncio.sleep(FINALIZE_WAIT_SECONDS)

            # Use recordings API to find the new recording(s) and save metadata + files
            try:
                async with g3.recordings.keep_updated_in_context():
                    await asyncio.sleep(1.0)
                    children = g3.recordings.children
                    if not children:
                        logger.warning("No recordings found on device after stopping.")
                        return
                    new_recording = None
                    for r in children:
                        if getattr(r, "uuid", None) not in existing_uuids:
                            new_recording = r
                            break
                    if new_recording is None:
                        new_recording = children[0]
                    rec_uuid = getattr(new_recording, "uuid", None)
                    rec_folder = session_folder / (rec_uuid or timestamp)
                    rec_folder.mkdir(parents=True, exist_ok=True)
                    meta = await save_recording_metadata(new_recording, rec_folder)
                    http_path = meta.get("get_http_path") or meta.get("get_rtsp_path") or None
                    http_base = None
                    http_url_attr = getattr(g3, "_http_url", None)
                    if http_url_attr:
                        http_base = http_url_attr
                    else:
                        http_base = f"http://{hostname}"
                    if http_path:
                        logger.info("Attempting to download files from http_base='%s' http_path='%s'", http_base, http_path)
                        await try_download_recording_files(http_base, http_path, rec_folder / "files")
                    else:
                        logger.info("No http_path available for recording; only metadata saved.")
            except Exception:
                logger.exception("Failed to fetch/save recordings after stopping.")
    logger.info("Done. Files saved under %s", session_folder.resolve())


def main():
    load_dotenv()
    hostname = G3_HOSTNAME
    if not hostname:
        raise RuntimeError("Set G3_HOSTNAME environment variable to the device hostname or IP.")
    out_base = DEFAULT_OUTPUT_BASE
    out_base.mkdir(parents=True, exist_ok=True)
    try:
        asyncio.run(start_and_run_recording(hostname, out_base))
    except Exception as e:
        logger.exception("Top-level exception: %s", e)


if __name__ == "__main__":
    main()
