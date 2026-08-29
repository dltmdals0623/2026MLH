from csicam import CsiCamCapture
import tracker as tr
import packet as pk
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
                print('\r'+"\033[31mCameraError: Failed to capture camera\033[0m",end='')
                return True
            elif cam_ret and self.start==False:
                input('\nPRESS ENTER 2 START')
                self.start=True
            src=cv2.GaussianBlur(src,(0,0),5)

            hsv = cv2.cvtColor(src,cv2.COLOR_BGR2HSV)
            gray = cv2.inRange(hsv, p.MIN_LINE_HSV, p.MAX_LINE_HSV)

            _, sobel=lane.sobel(gray)

            cam_h, cam_w = src.shape[:2]
            line_hulls = tr.extract_contours(sobel, min_area=p.MIN_DETECT_AREA)
            lines=lane.extract_lines(line_hulls,cam_h,cam_w//2)
            dst, c = lane.overlay_contours(src, lines,0,(0,255,255))

            if c is not None:
                cv2.line(dst,(cam_w//2, int(cam_h * p.CURRENT_LINE)), c,(  0,  0,255),1)
                theta = lane.get_theta((cam_w//2, int(cam_h * p.CURRENT_LINE)), c)
            if theta != 0.0:
                offset = int((abs(theta)**(2.2))*0.01)

            # if theta>0:
            #     offset = [(1, 512+delta//2), (2, 512), (3, 512), (4, 512+delta)]
            # else:
            #     offset = [(1, 512), (2, 512-delta//2), (3, 512-delta), (4, 512)]
            if offset is not None:
                if abs(offset) < 40:
                    offset=pk.serial_raw(offset+40)
                elif theta < 0:
                    offset=pk.serial_raw(254)
                else:
                    offset=pk.serial_raw(255)

            # pk_ret=pk.packet_write(*offset)
            # if pk_ret != 1:
            #     match(pk_ret):
            #         case 0:
            #             print("SERIAL NOT AVAILABLE",end='')
            #         case -2:
            #             print("Packet [START] ACK Timeout!",end='')
            #         case -3:
            #             print("Packet [END] ACK Timeout!",end='')

            cv2.imshow("hsv",gray)
            cv2.imshow("sobel", sobel)
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
