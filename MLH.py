import tracker as tr
import packet as pk
import param as p
import lane
import cv2
# import ik


class MLH:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)


    def main(self):
            theta=0.0
            cam_ret, src = self.cap.read()
            if not cam_ret:
                print("\033[31mCameraError: Failed to capture camera\033[0m")
                return False

            gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
            _, sobel=lane.sobel(gray)

            # dst = cv2.GaussianBlur(src, (0, 0), 10)
            # hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
            cam_h, cam_w = src.shape[:2]
            # line_mask = cv2.inRange(hsv, p.MIN_LINE_HSV, p.MAX_LINE_HSV)

            # line_mask[:int(cam_h * p.UPPER_LINE_MASK_LIMIT), :] = 0
            # line_mask[int(cam_h * p.LOWER_LINE_MASK_LIMIT):, :] = 0
            # line_mask[:, :int(cam_w * p.LEFT_LINE_MASK_LIMIT) ] = 0
            # line_mask[:, int(cam_w * p.RIGHT_LINE_MASK_LIMIT):] = 0

            # line_hulls = tr.extract_approxPolyDP(line_mask, min_area=p.MIN_DETECT_AREA)
            line_hulls = tr.extract_contours(sobel, min_area=p.MIN_DETECT_AREA)
            # line_hulls = tr.sort_contour_area(line_hulls)
            # line_hulls = tr.sort_contour_moment(line_hulls,cam_h, cam_w)
            lines=lane.extract_lines(line_hulls,cam_h,cam_w//2)
            dst, c = lane.overlay_contours(src, lines,0,(0,255,255))

            pk_ret=None

            if c is not None:
                cv2.line(dst,(cam_w//2, int(cam_h * p.CURRENT_LINE)), c,(  0,  0,255),1)    
                theta = lane.get_theta((cam_w//2, int(cam_h * p.CURRENT_LINE)), c)

                if abs(theta) <= 20:
                    pk_ret=pk.serial_raw(3)
                elif abs(theta) <= 30:
                    if theta < 0:
                        pk_ret=pk.serial_raw(2)
                    else:
                        pk_ret=pk.serial_raw(4)
                else:
                    if theta < 0:
                        pk_ret=pk.serial_raw(1)
                    else:
                        pk_ret=pk.serial_raw(5)

            cv2.imshow("sobel", sobel)
            cv2.imshow("dst", dst)

            print('\r'+f"THETA: {theta:>7.3f}\t{pk_ret}   ",end='')

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