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


class MLH:                                                                                          #멀티 레그드 허들 클래스
    def __init__(self):                                                                             #생성자
        logging.basicConfig(level=logging.INFO, format="[%(levelname)s %(lineno)d] %(message)s")    #로그 출력 형식 설정
        self.cap =   cv2.VideoCapture(0)                #카메라 연결
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -7.5)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.frame_cnt=0
        self.st  =   time.time()
        self.prev=0

        try:                                                                                        #시리얼 연결 시도
            self.ser = serial.Serial(port="COM4",
                                    baudrate=57600,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    bytesize=serial.EIGHTBITS,
                                    timeout=0)
            self.ser_enabled = True                                                                 #성공시 시리얼 사용
        except serial.SerialException as e:                                                         #시리얼 에러 발생 시
            logging.warning(f"SerialError: {str(e)}")                                               #경고레벨로 로그 출력
            self.ser_enabled = False                                                                #실패시 시리얼 미사용


    def main(self): #메인 함수
            fst=time.time()
            cam_ret, src = self.cap.read()                                                                          #카메라 사용 여부와 카메라 화면 추출
            if not cam_ret:                                                                                         #카메라가 비활성화 되어있다면
                logging.warning("CameraError: Failed to capture camera")                                                #경고레벨로 로그 출력
                return False                                                                                            #메인 함수 종료 리턴
            dst = cv2.GaussianBlur(src, (0, 0), 10)                                                                 #기본 화면에 강도 10의 블러 효과
            hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)                                                              #기본 화면의 BGR 색공간을 HSV 색공간으로 변환
            cam_h, cam_w = dst.shape[:2]                                                                  #카메라 화면의 높이와 너비 추출
            line_mask = cv2.inRange(hsv, p.MIN_LINE_HSV, p.MAX_LINE_HSV)                                  #라인 HSV 범위로 HSV 화면 이진화
            # line_mask[:int(cam_h*UPPER_LINE_MASK_LIMIT), :] = 0                                           #라인 감지 한계 위쪽 값 버림
            # line_mask[int(cam_h*LOWER_LINE_MASK_LIMIT):, :] = 0                                           #라인 감지 한계 아래쪽 값 버림

            line_hulls = tr.extract_approxPolyDP(line_mask, min_area=p.MIN_DETECT_AREA)
            line_hulls = tr.sort_contour_area(line_hulls)
            dst, c= lane.overlay_contours(dst,line_hulls,0,(255,255,  0),2)
            if c is not None:
                tmpmask = np.zeros_like(dst)
                cv2.line(tmpmask, (cam_w//2, int(cam_h*p.CURRENT_LINE)), c, (  0,  0,255), 2)
                dst= cv2.addWeighted(dst, 1.0, tmpmask, 1.0, 0)
            theta, linw_w = lane.get_theta((cam_w//2, cam_h *p.CURRENT_LINE),c)

            cv2.imshow("dst", dst)                                      #기본 입력 화면 출력
            cv2.imshow("line", line_mask)                             #라인 감지 화면 출력
            self.frame_cnt+=1
            et=time.time()

            if (et-self.st)//0.1!=self.prev:
                print('\r'+f"the: {theta:>7.3f}",end="")
                self.prev = (et-self.st)//0.1

            if et-self.st>=1.0:
                print(f" | fps: {self.frame_cnt/(et-self.st):.3f} | tpf: {et-fst:.3f}",end="")
                self.st=time.time()
                self.frame_cnt = 0

            key = cv2.waitKey(1)                                        #1틱마다 누른 키 감지
            if key == ord("q"):                                         #"Q"키를 눌렀다면
                print()
                return False                                                #메인 함수 종료 리턴

            return True                                                 #메인 함수 반복 리턴



    def run(self): #프로그램 실행 함수
        try:
            while True:                                 #무조건 반복
                if not self.main():                         #실행한 메인 함수에서 종료를 리턴했다면
                    logging.info("USSR Terminated")
                    break                                       #반복 종료
        except KeyboardInterrupt:                       #Ctrl C 입력 시
            logging.info("\nPressed 'Ctrl+C'")                #인포레벨로 로그 출력
        finally:                                        #로그 출력 후
            self.cap.release()                          #카메라 연결 해제
            cv2.destroyAllWindows()                     #모든 창 종료



if __name__ == "__main__":  #파일이 직접 실행되었다면
    mlh = MLH()
    mlh.run()