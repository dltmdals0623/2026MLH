import cv2
import numpy as np
import subprocess
import shlex
import threading



class CsiCamCapture:
    def __init__(self, cam_num, width=640, height=480, framerate=30):
        capture_command = (
            "rpicam-vid "
            "-n "
            "-t 0 "
            "-o - "
            "--flush "
            "--inline "
            "--codec mjpeg "
            f"--width {width} "
            f"--height {height} "
            f"--framerate {framerate} "
            f"--camera {cam_num} "
        )
        self.process = subprocess.Popen(
            shlex.split(capture_command),
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0
        )

        self.frame_buffer = bytearray()
        self.frame = None

        self.thread_running = True
        self.thread_lock = threading.Lock()
        self.thread = threading.Thread(
            target=self._capture_thread,
            daemon=True
        )
        self.thread.start()



    def _capture_thread(self):
        while self.thread_running:
            self.frame_buffer.extend(self.process.stdout.read(65536))

            eoi = self.frame_buffer.rfind(b"\xff\xd9")
            if eoi == -1:
                continue

            soi = self.frame_buffer.rfind(b"\xff\xd8", 0, eoi)
            if soi == -1:
                del self.frame_buffer[: eoi + 2]
                continue

            frame = bytes(self.frame_buffer[soi : eoi + 2])
            del self.frame_buffer[: eoi + 2]
            frame = cv2.imdecode(np.frombuffer(frame, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue


            with self.thread_lock:
                self.frame = frame



    def read(self):
        with self.thread_lock:
            if self.frame is None:
                return False, None
            else:
                return True, cv2.flip(self.frame.copy(),-1)



    def release(self):
        self.thread_running = False

        self.process.terminate()

        if self.thread.is_alive():
            self.thread.join(timeout=1)

        self.process.wait()