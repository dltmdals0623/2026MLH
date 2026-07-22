import cv2
import serial
import logging
import time
import numpy as np

import tracker as tr
import packet as pk
import lane
import param as p
import ik


class MLH:
    def __init__(self):
        logging.basicConfig(level=logging.INFO, format="[%(levelname)s %(lineno)d] %(message)s")
        self.cap =   cv2.VideoCapture(0)                #카메라 연결
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, -7.5)
        # self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.frame_cnt=0
        self.st=time.time()
        self.prev=0

        try:
            self.ser = serial.Serial(port="COM4",
                                    baudrate=57600,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    bytesize=serial.EIGHTBITS,
                                    timeout=0)
            self.ser_enabled = True
        except serial.SerialException as e:
            logging.warning(f"SerialError: {str(e)}")
            self.ser_enabled = False


    def main(self): #메인 함수
            fst=time.time()
            cam_ret, src = self.cap.read()
            if not cam_ret:
                logging.warning("CameraError: Failed to capture camera")
                return False
            dst = cv2.GaussianBlur(src, (0, 0), 10)
            hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
            cam_h, cam_w = dst.shape[:2]
            line_mask = cv2.inRange(hsv, p.MIN_LINE_HSV, p.MAX_LINE_HSV)

            line_hulls = tr.extract_approxPolyDP(line_mask, min_area=p.MIN_DETECT_AREA)
            line_hulls = tr.sort_contour_area(line_hulls)
            dst, c= lane.overlay_contours(dst,line_hulls,0,(255,255,  0),2)
            if c is not None:
                tmpmask = np.zeros_like(dst)
                cv2.line(tmpmask, (cam_w//2, int(cam_h*p.CURRENT_LINE)), c, (  0,  0,255), 2)
                dst= cv2.addWeighted(dst, 1.0, tmpmask, 1.0, 0)
            theta, linw_w = lane.get_theta((cam_w//2, cam_h *p.CURRENT_LINE),c)

            ik.cal_offset(theta)

            cv2.imshow("dst", dst)
            cv2.imshow("line", line_mask)
            self.frame_cnt+=1
            et=time.time()

            if (et-self.st)//0.1!=self.prev:
                print('\r'+f"the: {theta:>7.3f}",end="")
                self.prev = (et-self.st)//0.1

            if et-self.st>=1.0:
                print(f" | fps: {self.frame_cnt/(et-self.st):.3f} | tpf: {et-fst:.3f}",end="")
                self.st=time.time()
                self.frame_cnt = 0

            key = cv2.waitKey(1)
            if key == ord("q"):
                print()
                return False

            return True



    def run(self):
        try:
            while True:
                if not self.main():
                    logging.info("USSR Terminated")
                    break
        except KeyboardInterrupt:
            logging.info("\nPressed 'Ctrl+C'")
        finally:
            self.cap.release()
            cv2.destroyAllWindows()



if __name__ == "__main__":
    mlh = MLH()
    mlh.run()