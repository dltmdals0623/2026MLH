from csicam import CsiCamCapture
import tracker as tr
import packet as pk
import numpy as np
import param as p
import lane
import cv2
import os


class MLH:
    def __init__(self):
        if os.name == 'nt':
            self.cap = cv2.VideoCapture(0)
        else:
            self.cap = CsiCamCapture(0)
        self.start=False


    def main(self):
            theta=0.0
            offset=None
            cam_ret, src = self.cap.read()
            if not cam_ret:
                print('\r'+"\033[31mFinding Camera..\033[0m",end='')
                return True
            elif cam_ret and self.start==False:
                input('\r'+'\033[32mCamera Detected!\033[0m\n\nPRESS ENTER 2 START')
                self.start=True
            src=cv2.GaussianBlur(src,(0,0),3)

            hsv = cv2.cvtColor(src,cv2.COLOR_BGR2HSV)
            line = cv2.inRange(hsv, p.MIN_LINE_HSV, p.MAX_LINE_HSV)
            obj = cv2.inRange(hsv, p.MIN_OBJ_HSV, p.MAX_OBJ_HSV)

            cam_h, cam_w = src.shape[:2]
            line_hulls = tr.extract_contours(line, min_area=p.MIN_DETECT_AREA)
            obj_hulls = tr.extract_contours(obj, min_area=p.MIN_DETECT_AREA)
            lines=lane.extract_lines(line_hulls,cam_h,cam_w//2)
            dst, c = lane.overlay_contours(src, lines, 0, (  0, 255, 255), 0)
            dst, c_obj = lane.overlay_contours(dst, obj_hulls, 0, (   0, 255,   0), 1)

            if c_obj is not None:
                c=c_obj

            if c is not None:
               cv2.line(dst,(cam_w//2, int(cam_h * p.CURRENT_LINE)), c,(  0,  0, 255),1)
               theta = lane.get_theta((cam_w//2, int(cam_h * p.CURRENT_LINE)), c)
            if theta != 0.0:
                offset = int((theta*abs(theta)**1.4)*0.05)

            if offset is not None:

                # if abs(offset) <= 60:
                #     offset=pk.serial_raw(np.clip(offset,-40,40)+512)
                # elif theta < 0:
                #     offset=pk.serial_raw(254+512)
                # else:
                #     offset=pk.serial_raw(255+512)

                if abs(theta) <= 20:
                    offset=pk.serial_raw(3)
                elif abs(theta) <= 30:
                    if theta < 0:
                        offset=pk.serial_raw(2)
                    else:
                        offset=pk.serial_raw(4)
                else:
                    if theta < 0:
                        offset=pk.serial_raw(1)
                    else:
                        offset=pk.serial_raw(5)

            cv2.imshow("hsv",line)
            cv2.imshow("obj", obj)
            cv2.imshow("dst", dst)


            print('\r'+f"THETA: {theta:>7.3f}    {offset}    ",end='')

            key = cv2.waitKey(1)
            if key == ord("q"):
                print()
                print("\033[32mUSSR Terminated\033[0m")
                return False

            return True


    def run(self):
        try:
            while self.main():
                pass
        except KeyboardInterrupt:
            print("\n\033[32mPressed 'Ctrl+C'\033[0m")
        finally:
            self.cap.release()
            cv2.destroyAllWindows()



if __name__ == "__main__":
    mlh = MLH()
    mlh.run()
