#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import time
import threading
import argparse

class CameraRecorder:
    def __init__(self, device_id=0, frame_rate=30.0, width=640, height=480, frame_id="camera_frame",
                 base_dir="/mnt/bikelab_data"):
        # Parameters (kept similar to ROS params)
        self.device_id = int(device_id)
        self.frame_rate = float(frame_rate)
        self.width = int(width)
        self.height = int(height)
        self.frame_id = str(frame_id)

        # Output directory: /mnt/bikelab_data/camera_$(date +%Y%m%d_%H%M%S)
        ts_dir = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self.out_dir = os.path.join(base_dir, f"camera_{ts_dir}")
        self.frames_dir = os.path.join(self.out_dir, "frames")
        os.makedirs(self.frames_dir, exist_ok=True)

        # CSV for timestamps
        self.csv_path = os.path.join(self.out_dir, "timestamps.csv")
        self.csv_f = open(self.csv_path, "w", buffering=1)  # line-buffered
        self.csv_f.write("frame_idx,unix_ns,filename\n")

        # ---- Open camera (V4L2 + MJPG) ----
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)

        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera device {self.device_id}")

        # Latest frame buffer (BGR)
        self._lock = threading.Lock()
        self._latest = None
        self._running = True

        # Grab thread stats
        self._grab_cnt = 0
        self._grab_t0 = time.time()

        # Recorder counters
        self._frame_idx = 1

        # --- video writer (MJPG AVI) ---
        self.video_path = os.path.join(self.out_dir, "video_mjpg.avi")
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        self.writer = cv2.VideoWriter(self.video_path, fourcc, self.frame_rate, (self.width, self.height))
        if not self.writer.isOpened():
            raise RuntimeError(f"Failed to open VideoWriter: {self.video_path}")

        # Start grab thread
        self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._grab_thread.start()

        # Timer loop period for "publish" (now: save)
        self.period = 1.0 / (self.frame_rate if self.frame_rate > 0 else 30.0)

        print(
            f"Camera recorder started (V4L2+MJPG): /dev/video{self.device_id}, "
            f"{self.width}x{self.height}@{int(self.frame_rate)}"
        )
        print(f"Output dir: {self.out_dir}")

        t0 = time.perf_counter()
        self._next_fps_log = t0 + 5.0
        self._grab_cnt = 0
        self._grab_t0 = t0
                

    def _grab_loop(self):
        while self._running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.001)
                continue

            with self._lock:
                self._latest = frame

            self._grab_cnt += 1
            now = time.perf_counter()
            if now >= self._next_fps_log:
                elapsed = now - self._grab_t0
                fps = self._grab_cnt / elapsed if elapsed > 0 else 0.0
                print(f"grab fps ~ {fps:.1f} (over {elapsed:.1f}s)")
                self._grab_cnt = 0
                self._grab_t0 = now
                self._next_fps_log = now + 5.0

    @staticmethod
    def _unix_time_ns() -> int:
        # wall clock nanoseconds (good enough for aligning other logs if they use unix time)
        return time.time_ns()

    def _save_latest(self):
        """Save at fixed rate: write one frame into MJPG AVI + timestamps.csv."""
        with self._lock:
            if self._latest is None:
                return
            frame = self._latest.copy()

        unix_ns = self._unix_time_ns()

        # 防御：确保尺寸匹配 VideoWriter（避免崩或写不进去）
        h, w = frame.shape[:2]
        if (w, h) != (self.width, self.height):
            frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)

        # 写入视频（顺序写）
        self.writer.write(frame)

        # 记录时间戳：frame_idx 对应视频的第几帧
        # filename 写视频名即可（或者留空也行）
        self.csv_f.write(f"{self._frame_idx},{unix_ns},video_mjpg.avi\n")

        self._frame_idx += 1

    def run(self):
        """Main loop: keep the same fixed-rate publish timer concept, now saving."""
        next_t = time.perf_counter()
        try:
            while True:
                now = time.perf_counter()
                if now >= next_t:
                    self._save_latest()
                    next_t += self.period
                else:
                    # sleep a bit to reduce CPU
                    time.sleep(min(0.001, next_t - now))
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def close(self):
        self._running = False
        try:
            self._grab_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            if self.cap.isOpened():
                self.cap.release()
        except Exception:
            pass

        # try:
        #     if hasattr(self, "writer"):
        #         self.writer.release()
        # except Exception:
        #     pass

        try:
            self.csv_f.close()
        except Exception:
            pass

        print("Recorder stopped cleanly.")


def main():
    parser = argparse.ArgumentParser(description="ROS-free camera recorder (V4L2+MJPG) saving frames to disk.")
    parser.add_argument("--device_id", type=int, default=0)
    parser.add_argument("--frame_rate", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--frame_id", type=str, default="camera_frame")
    parser.add_argument("--base_dir", type=str, default="/mnt/bikelab_data")
    args = parser.parse_args()

    rec = CameraRecorder(
        device_id=args.device_id,
        frame_rate=args.frame_rate,
        width=args.width,
        height=args.height,
        frame_id=args.frame_id,
        base_dir=args.base_dir,
    )
    rec.run()


if __name__ == "__main__":
    main()
