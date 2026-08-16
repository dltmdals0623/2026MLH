from csicam import CsiCamCapture as ccc
import tracker as tr
import packet as pk
import param as p
import lane
import cv2
# import ik


class MLH:
    def __init__(self):
        try:
            self.cap = cv2.VideoCapture(0)
        except:
            self.cap = ccc(0)
        self.gait_phase = 0  # 0: [FL, BR] 스윙 / 1: [FL, BR] 스탠스(밀기), 2: [FR, BL] 스윙 ...
        self.last_theta = 0.0


    def main(self):
            theta=0.0
            cam_ret, src = self.cap.read()
            if not cam_ret:
                print("\033[31mCameraError: Failed to capture camera\033[0m")
                return False
            
            dst = cv2.GaussianBlur(src, (0, 0), 10)
            hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
            cam_h, cam_w = dst.shape[:2]
            line_mask = cv2.inRange(hsv, p.MIN_LINE_HSV, p.MAX_LINE_HSV)

            line_mask[:int(cam_h * p.UPPER_LINE_MASK_LIMIT), :] = 0
            line_mask[int(cam_h * p.LOWER_LINE_MASK_LIMIT):, :] = 0
            line_mask[:, :int(cam_w * p.LEFT_LINE_MASK_LIMIT) ] = 0
            line_mask[:, int(cam_w * p.RIGHT_LINE_MASK_LIMIT):] = 0

            line_hulls = tr.extract_approxPolyDP(line_mask, min_area=p.MIN_DETECT_AREA)
            line_hulls = tr.sort_contour_area(line_hulls)
            dst, c = lane.vsl_contours(dst, line_hulls,0,(0,255,255))

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


            # if self.gait_phase in [1, 3]:  # Stance(지지 및 밀기) 구간
            # # V_x(전진 속도)와 W_z(theta 기반 회전 속도) 조합
            #     v_x = 0.05
            #     w_z = theta * 0.02
            # else:
            #     v_x = 0.0
            #     w_z = 0.0

            # offset = ik.cal_offset_with_cmd_vel(range(1, 13), v_x, 0.0, w_z)

            cv2.imshow("dst", dst)
            cv2.imshow("line", line_mask)

            print('\r'+f"THETA: {theta:>7.3f}\t{pk_ret}",end='\t')

            # print('\r'+f"THETA: {theta:>7.3f}\t{offset}",end='\t')

            # pk_ret, pk_key, pk_data=pk.packet_write(*offset)
            # if pk_ret != 1:
            #     match(pk_ret):
            #         case  0: print( "\033[31mSERIAL NOT AVAILABLE\033[0m",end='                    ')
            #         # case -1: print(f"\033[31mPacket [{pk_key}:{pk_data}] ACK Timeout!\033[0m",end='')
            #         case -2: print( "\033[31mPacket [START] ACK Timeout!\033[0m",end='             ')
            #         case -3: print( "\033[31mPacket [END] ACK Timeout!\033[0m",end='               ')
            # else:            print( "\033[32mPacket Sending SUCCEED\033[0m",end='                  ')

            key = cv2.waitKey(1)
            if key == ord("q"):
                print()
                print("\033[32mUSSR Terminated\033[0m")
                return False

            self.gait_phase = (self.gait_phase + 1) % 4
            
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