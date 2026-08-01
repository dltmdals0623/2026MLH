import cv2
import time
import serial
import logging
import numpy as np

import tracker as tr
import packet as pk
import param as p
import lane
import ik



class MLH:
    def __init__(self):
        logging.basicConfig(level=logging.INFO, format="[%(levelname)s %(lineno)d] %(message)s")
        self.cap =   cv2.VideoCapture(1)                #카메라 연결
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, -7.5)
        # self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.frame_cnt=0
        self.st=time.time()
        self.prev=0



    def main(self):
            fst=time.time()
            cam_ret, src = self.cap.read()
            if not cam_ret:
                print("CameraError: Failed to capture camera")
                return False
            
            dst = cv2.GaussianBlur(src, (0, 0), 10)
            hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
            cam_h, cam_w = dst.shape[:2]
            line_mask = cv2.inRange(hsv, p.MIN_LINE_HSV, p.MAX_LINE_HSV)

            line_mask[:int(cam_h * p.UPPER_LINE_MASK_LIMIT), :] = 0
            line_mask[int(cam_h * p.LOWER_LINE_MASK_LIMIT):, :] = 0
            line_mask[:, :int(cam_w * p.LEFT_LINE_MASK_LIMIT)] = 0
            line_mask[:, int(cam_w * p.RIGHT_LINE_MASK_LIMIT):] = 0

            dst, c = lane.vsl_sobel_lanes(dst, line_mask)

            # 2. c를 기반으로 theta 및 linw_w 계산
            if c is not None:
                theta, line_w = lane.get_theta((cam_w//2, int(cam_h * p.CURRENT_LINE)), c)
            else:
                theta, line_w = 0.0, 0

            cv2.imshow("dst", dst)
            self.frame_cnt += 1

            print('\r'+f"THETA: {theta:>7.3f}", end='\t\t')

            cv2.imshow("dst", dst)
            cv2.imshow("line", line_mask)
            # cv2.imshow("sobel", sobel)
            self.frame_cnt+=1
            et=time.time()

            print('\r'+f"THETA: {theta:>7.3f}",end='\t\t')

            print(ik.cal_offset(theta),end='\t\t')
            pk_ret, pk_key, pk_data=pk.packet_write(*ik.cal_offset(theta))
            if pk_ret != 1:
                match(pk_ret):
                    case 0:
                        print("SERIAL NOT AVAILABLE",end='')
                    case -1:
                        print(f"Packet [{pk_key}:{pk_data}] ACK Timeout!",end='')
                    case -2:
                        print("Packet [START] ACK Timeout!",end='')
                    case -3:
                        print("Packet [END] ACK Timeout!",end='')

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