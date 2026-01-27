#!/usr/bin/env python3
import asyncio
import os
import time
from g3pylib import connect_to_glasses

G3_HOST = os.environ.get("G3_HOSTNAME", "192.168.75.51")  

async def enable_ntp_and_wait(hostname: str, timeout_s: int = 60):
    async with connect_to_glasses.with_hostname(hostname, using_zeroconf=False) as g3:
        ok = await g3.system.use_ntp(True)
        print("use_ntp(True) ->", ok)
        start = time.time()
        while time.time() - start < timeout_s:
            synced = await g3.system.get_ntp_is_synchronized()
            print("ntp synchronized:", synced)
            if synced:
                break
            await asyncio.sleep(1)
        print("final synced:", await g3.system.get_ntp_is_synchronized())
        print("device time:", await g3.system.get_time())

if __name__ == "__main__":
    asyncio.run(enable_ntp_and_wait(G3_HOST))
