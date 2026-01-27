"""
save_recording_with_stop_on_ctrlc_simple.py

Connects to Glasses3, waits for (or enables) NTP sync, starts recording, runs until you press Ctrl+C,
then stops recording.

No metadata saving, no file download.

Usage:
  - Set environment variable G3_HOSTNAME to the glasses hostname or IP (or pass it in code).
  - Run: python save_recording_with_stop_on_ctrlc_simple.py

Dependencies:
  pip install glasses3-pylib python-dotenv
"""
import asyncio
import logging
import os
import signal
from datetime import datetime, timezone, timedelta
from pathlib import Path

from dotenv import load_dotenv
from g3pylib import connect_to_glasses

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("g3-save")

# Configuration
DEFAULT_OUTPUT_BASE = Path(os.environ.get("OUTPUT_DIR", "./recordings"))  # 现在仅作占位，不实际写文件
G3_HOSTNAME = os.environ.get("G3_HOSTNAME")  # required, unless you change the code to use zeroconf
NTP_WAIT_TIMEOUT = float(os.environ.get("NTP_WAIT_TIMEOUT", "10"))  # seconds
CLOCK_OFFSET_THRESHOLD_SECONDS = float(os.environ.get("CLOCK_OFFSET_THRESHOLD_SECONDS", "0.5"))
FINALIZE_WAIT_SECONDS = float(os.environ.get("FINALIZE_WAIT_SECONDS", "3.0"))  # wait for device to finalize internally


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

async def run_calibration(g3, max_attempts: int = 3, marker_wait_s: float = 3.0) -> bool:
    """
    Marker-based calibration BEFORE recording.

    Workflow:
      1) Ask operator to position the official target at ~50-100 cm.
      2) Enable marker detection (emit-markers) for a few seconds.
      3) (Optional) Listen for marker signal and print marker pose if available.
      4) When ready, trigger calibrate(true).
    """
    loop = asyncio.get_running_loop()

    for attempt in range(1, max_attempts + 1):
        logger.info("Calibration attempt %d / %d", attempt, max_attempts)
        logger.info(
            "Place Tobii calibration target in front of participant (~50-100 cm). "
            "Ask participant to fixate the center dot. Hold still."
        )

        # Step 1: enable marker detection for ~3 seconds (or until calibration completes)
        try:
            # Some SDKs expose this as g3.calibrate.emit_markers() or a generic action call.
            # We try both patterns to be robust.
            if hasattr(g3, "calibrate") and hasattr(g3.calibrate, "emit_markers"):
                await g3.calibrate.emit_markers()
                logger.info("Marker detection enabled (emit-markers).")
            elif hasattr(g3, "calibrate") and hasattr(g3.calibrate, "emitMarkers"):
                await g3.calibrate.emitMarkers()
                logger.info("Marker detection enabled (emit-markers).")
            else:
                # If the SDK uses a generic action interface, you can adapt here.
                logger.info("emit-markers API not found in this g3pylib build; continuing without marker stream.")
        except Exception as e:
            logger.warning("Failed to enable marker detection (emit-markers): %s", e)

        # Step 2: optionally read marker signal if the SDK provides a subscription method
        marker_seen = False
        try:
            # Typical pattern (if available): subscribe_to_marker() returns (queue, unsubscribe)
            if hasattr(g3, "calibrate") and hasattr(g3.calibrate, "subscribe_to_marker"):
                q, unsub = await g3.calibrate.subscribe_to_marker()
                logger.info("Subscribed to calibrate:marker signal (waiting up to %.1fs)...", marker_wait_s)
                try:
                    msg = await asyncio.wait_for(q.get(), timeout=marker_wait_s)
                    # Expected body: [timestamp, [x,y,z], [u,v]]
                    logger.info("Marker signal: %s", msg)
                    marker_seen = True
                except asyncio.TimeoutError:
                    logger.warning("No marker signal received within %.1fs.", marker_wait_s)
                finally:
                    try:
                        await unsub
                    except Exception:
                        pass
            else:
                # No subscription available; just wait a moment so emit-markers can do work
                await asyncio.sleep(marker_wait_s)
        except Exception as e:
            logger.warning("Marker subscription not available / failed: %s", e)

        # Step 3: trigger calibration
        logger.info("Press ENTER to trigger calibration now...")
        await loop.run_in_executor(None, input, "")

        try:
            # New method: calibrate(true) -> returns detailed result, e.g., "success"
            if hasattr(g3, "calibrate") and hasattr(g3.calibrate, "calibrate"):
                result = await g3.calibrate.calibrate(True)
            elif hasattr(g3, "calibrate") and hasattr(g3.calibrate, "run"):
                # Old method: run() -> True/False
                result = await g3.calibrate.run()
            else:
                raise RuntimeError("No calibration API found (expected g3.calibrate.calibrate or g3.calibrate.run).")

            logger.info("Calibration result: %r", result)

            if isinstance(result, str):
                if result.lower().startswith("success"):
                    logger.info("Calibration SUCCESS.")
                    return True
            elif isinstance(result, bool):
                if result:
                    logger.info("Calibration SUCCESS.")
                    return True

            logger.warning("Calibration not successful (result=%r). Adjust target/pose and retry.", result)

        except Exception as e:
            logger.exception("Calibration attempt failed: %s", e)

    logger.error("Calibration failed after %d attempts.", max_attempts)
    return False

async def start_and_run_recording(hostname: str, output_base: Path):
    """
    High-level flow (simplified):
    - Connect
    - Ensure NTP sync or enable NTP
    - If NTP sync still not achieved within timeout, set device time to current local system time
      using g3.system.set_time(...)
    - Start recorder (subscribe to started signal)
    - Run until Ctrl+C (SIGINT)
    - Stop recorder and finalize

    No metadata saving, no file download.
    """
    if not hostname:
        raise RuntimeError("Provide hostname (set G3_HOSTNAME env or call function with hostname).")

    logger.info("Output base (not used for saving in this simple script): %s", output_base.resolve())

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
                        logger.warning(
                            "Clock offset after set_time is %s (threshold %ss).",
                            offset_after,
                            CLOCK_OFFSET_THRESHOLD_SECONDS,
                        )
                except Exception:
                    logger.debug("Could not verify device time after set_time")
            except Exception as e:
                logger.exception("Failed to set device time from local system time: %s", e)
        
        # --- Calibration BEFORE recording ---
        logger.info("Starting calibration workflow before recording...")
        calib_ok = await run_calibration(g3, max_attempts=3)
        if not calib_ok:
            logger.error("Calibration failed. Recording will NOT start.")
            return
        logger.info("Calibration completed successfully. Proceeding to recording.")
        # --- End calibration ---

        # Prepare to subscribe to started signal
        queue, unsubscribe = await g3.recorder.subscribe_to_started()
        logger.info("Subscribed to recorder 'started' signal")

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

            # Give the device a moment to finalize internally
            await asyncio.sleep(FINALIZE_WAIT_SECONDS)

    logger.info("Done. Recording stopped.")


def main():
    load_dotenv()
    hostname = os.environ.get("G3_HOSTNAME") or "192.168.1.166"

    if not hostname:
        raise RuntimeError("Set G3_HOSTNAME environment variable to the device hostname or IP.")
    out_base = DEFAULT_OUTPUT_BASE
    out_base.mkdir(parents=True, exist_ok=True)  # 不再写入其中，只是保证目录存在（可按需删掉）
    try:
        asyncio.run(start_and_run_recording(hostname, out_base))
    except Exception as e:
        logger.exception("Top-level exception: %s", e)


if __name__ == "__main__":
    main()
