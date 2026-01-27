#!/usr/bin/env python3
"""
sync_g3_time.py

Connects to a Tobii Glasses3 device (via g3pylib) and synchronizes its clock.

Behavior:
- Uses G3_HOSTNAME env var by default (or pass --hostname).
- By default prints device time, host time and offset.
- --sync-now will set the device clock to either:
    * the time returned by a local NTP server you specify (--ntp-server), OR
    * the local host UTC time (if no ntp server provided).
- --enable-ntp will enable NTP on the Glasses3 device after setting time and optionally wait until device.get_ntp_is_synchronized() is True.
- Because g3pylib does not provide an API to set the NTP server address on the device, to have the glasses use a particular NTP server automatically you must configure the device's NTP server via its OS/UI or ensure the default NTP configuration on the device resolves to your local NTP server.

Dependencies:
  pip install g3pylib ntplib aiohttp

Examples:
  # Print device vs host time
  G3_HOSTNAME=tg03b-080200045321 python sync_g3_time.py

  # Sync device to local NTP server (192.168.1.10) immediately
  G3_HOSTNAME=tg03b-080200045321 python sync_g3_time.py --sync-now --ntp-server 192.168.1.10

  # Sync device to host UTC
  python sync_g3_time.py --hostname tg03b-080200045321 --sync-now

  # Sync then enable NTP on device and wait for it to report synchronized (timeout 120s)
  python sync_g3_time.py --hostname tg03b-080200045321 --sync-now --ntp-server 192.168.1.10 --enable-ntp --wait-sync --wait-timeout 120
"""
from __future__ import annotations

import argparse
import asyncio
import os
import sys
import time
from datetime import datetime, timezone, timedelta
from typing import Optional

import ntplib
from g3pylib import connect_to_glasses


def get_ntp_time_sync(server: str, timeout: float = 5.0) -> datetime:
    """
    Query an NTP server (synchronously) and return a timezone-aware UTC datetime.
    May raise ntplib.NTPException or socket errors on failure.
    """
    client = ntplib.NTPClient()
    # ntplib returns tx_time as seconds since epoch (float)
    resp = client.request(server, version=3, timeout=timeout)
    return datetime.fromtimestamp(resp.tx_time, tz=timezone.utc)


async def get_ntp_time(server: str, timeout: float = 5.0) -> datetime:
    """
    Async wrapper for get_ntp_time_sync using a threadpool.
    """
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, get_ntp_time_sync, server, timeout)


async def run(
    hostname: str,
    sync_now: bool,
    ntp_server: Optional[str],
    enable_ntp: bool,
    wait_sync: bool,
    wait_timeout: int,
):
    print(f"Connecting to Glasses3 '{hostname}'...")
    try:
        async with connect_to_glasses.with_hostname(hostname) as g3:
            print("Connected.")
            # Read device state
            try:
                device_time = await g3.system.get_time()
            except Exception as e:
                print("Error reading device time:", e)
                return

            try:
                device_tz = await g3.system.get_timezone()
            except Exception:
                device_tz = "<unknown>"

            try:
                ntp_enabled = await g3.system.get_ntp_is_enabled()
            except Exception:
                ntp_enabled = None

            try:
                ntp_synced = await g3.system.get_ntp_is_synchronized()
            except Exception:
                ntp_synced = None

            host_time = datetime.now(timezone.utc)

            print(f"Device time (reported): {device_time.isoformat()}")
            print(f"Device timezone: {device_tz}")
            print(f"Device NTP enabled: {ntp_enabled}")
            print(f"Device NTP synchronized: {ntp_synced}")
            print(f"Host time (UTC): {host_time.isoformat()}")

            # Compute offset device - host in seconds (device_time assumed UTC per API)
            try:
                if device_time.tzinfo is None:
                    device_time_utc = device_time.replace(tzinfo=timezone.utc)
                else:
                    device_time_utc = device_time.astimezone(timezone.utc)
                offset = (device_time_utc - host_time).total_seconds()
                print(f"Clock offset (device - host) = {offset:.3f} seconds")
            except Exception as e:
                print("Could not compute offset:", e)
                offset = None

            if sync_now:
                # Decide source of time: remote ntp server or host
                if ntp_server:
                    print(f"Querying NTP server {ntp_server} for time...")
                    try:
                        ntp_time = await get_ntp_time(ntp_server)
                        new_time = ntp_time
                        print(f"NTP server time (UTC): {new_time.isoformat()}")
                    except Exception as e:
                        print(f"Failed to query NTP server {ntp_server}: {e}")
                        print("Falling back to host UTC time.")
                        new_time = datetime.now(timezone.utc)
                else:
                    new_time = datetime.now(timezone.utc)
                    print(f"Using host UTC time: {new_time.isoformat()}")

                # Set device clock
                try:
                    ok = await g3.system.set_time(new_time)
                    print(f"set_time -> {ok}")
                except Exception as e:
                    print("Failed to set device time:", e)
                    return

                # Read back device time to compute new offset
                try:
                    time.sleep(0.2)  # small pause so device can update, if needed
                    device_time2 = await g3.system.get_time()
                    if device_time2.tzinfo is None:
                        device_time2_utc = device_time2.replace(tzinfo=timezone.utc)
                    else:
                        device_time2_utc = device_time2.astimezone(timezone.utc)
                    new_offset = (device_time2_utc - datetime.now(timezone.utc)).total_seconds()
                    print(f"Device time after set: {device_time2.isoformat()}")
                    print(f"New offset (device - host) = {new_offset:.3f} seconds")
                except Exception as e:
                    print("Failed to read device time after set:", e)

            # Optionally enable/disable NTP on device
            if enable_ntp:
                try:
                    ok = await g3.system.use_ntp(True)
                    print(f"use_ntp(True) -> {ok}")
                except Exception as e:
                    print("Failed to enable NTP on device:", e)
                if wait_sync:
                    # Poll until device reports NTP synchronized or timeout
                    deadline = time.time() + wait_timeout
                    while time.time() < deadline:
                        try:
                            synced = await g3.system.get_ntp_is_synchronized()
                        except Exception as e:
                            print("Error while checking NTP sync:", e)
                            synced = False
                        print(f"NTP synchronized: {synced}")
                        if synced:
                            print("Device reports NTP synchronized.")
                            break
                        await asyncio.sleep(2)
                    else:
                        print(f"Timeout: device did not report NTP synchronized within {wait_timeout} seconds.")
            elif enable_ntp is False:
                try:
                    ok = await g3.system.use_ntp(False)
                    print(f"use_ntp(False) -> {ok}")
                except Exception as e:
                    print("Failed to disable NTP on device:", e)

            print("Done.")
    except Exception as e:
        print("Connection error:", e)


def parse_args():
    parser = argparse.ArgumentParser(description="Sync Tobii Glasses3 time to a local NTP server or host time.")
    parser.add_argument("--hostname", default=os.getenv("G3_HOSTNAME"), help="Glasses3 hostname (serial) or IP. Also read from env G3_HOSTNAME.")
    parser.add_argument("--sync-now", action="store_true", help="Immediately set device clock to NTP time (if --ntp-server provided) or to host UTC.")
    parser.add_argument("--ntp-server", default="192.168.75.158", help="Local NTP server IP/host to query (e.g., another RPi). If provided and --sync-now, script will fetch time from this server and set the device clock to it.")
    parser.add_argument("--enable-ntp", default="True", help="Enable NTP on the device after sync.")
    parser.add_argument("--disable-ntp", dest="enable_ntp", action="store_false", help="Disable NTP on the device.")
    parser.add_argument("--wait-sync", action="store_true", help="After enabling NTP, wait for the device to report it is synchronized.")
    parser.add_argument("--wait-timeout", type=int, default=120, help="Seconds to wait for device NTP sync when --wait-sync is used.")
    parser.set_defaults(enable_ntp=None)
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.hostname:
        print("No hostname provided. Set G3_HOSTNAME or pass --hostname.")
        sys.exit(1)
    asyncio.run(
        run(
            hostname=args.hostname,
            sync_now=args.sync_now,
            ntp_server=args.ntp_server,
            enable_ntp=args.enable_ntp,
            wait_sync=args.wait_sync,
            wait_timeout=args.wait_timeout,
        )
    )


if __name__ == "__main__":
    main()
