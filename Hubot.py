#!/usr/bin/python3

### Hubot
# Version 2.11.2

### Patch note



import numpy as np
import cv2
import serial
import logging

import tracker as tr



class Hubot:                                                                                        #휴봇 클래스
    def __init__(self):                                                                             #생성자
        logging.basicConfig(level=logging.INFO, format="[%(levelname)s %(lineno)d] %(message)s")    #로그 출력 형식 설정

        try:                                        #시리얼 연결 시도
            self.ser = serial.Serial(port="COM4",
                                    baudrate=57600,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    bytesize=serial.EIGHTBITS,
                                    timeout=0)
            self.ser_enabled = True                     #성공시 시리얼 사용
        except serial.SerialException as e:             #시리얼 에러 발생 시
            logging.warning(f"SerialError: {str(e)}")   #경고레벨로 로그 출력
            self.ser_enabled = False                    #실패시 시리얼 미사용

        self.cap =   cv2.VideoCapture(0)                #카메라 연결
        self.cam_w = 0                                  #카메라 화면 너비
        self.cam_h = 0                                  #카메라 화면 높이

        self.MIN_DETECT_WIDTH =  30                     #반영할 사각형 최소 너이
        self.MIN_DETECT_HEIGHT = 10                     #반영할 사각형 최소 높이
        self.MIN_DETECT_AREA =   5000                   #반영할 컨투어 최소 넓이

        self.MIN_LINE_HSV =   np.array([20, 20, 20])    #라인 최소 HSV
        self.MAX_LINE_HSV =   np.array([35, 255, 255])  #라인 최대 HSV
        self.MIN_OBJECT_HSV = np.array([100, 100, 50])  #물체 최소 HSV
        self.MAX_OBJECT_HSV = np.array([160, 255, 150]) #물체 최대 HSV

        self.UPPER_LINE_MASK_LIMIT = 0.00               #라인 감지 한계_상
        self.LOWER_LINE_MASK_LIMIT = 0.90               #라인 감지 한계_하
        self.left_line_mask_limit  = 0.10               #라인 감지 한계_좌
        self.right_line_mask_limit = 0.10               #라인 감지 한계_우
        self.CURRENT_LINE_POS =      1.50               #로봇 추정 위치 (세타값 중심점)

        self.UPPER_OBJECT_MASK_LIMIT = 0.50             #물체 감지 한계_상
        self.LOWER_OBJECT_MASK_LIMIT = 0.75             #물체 감지 한계_하
        self.LEFT_OBJECT_MASK_LIMIT = 0.20              #물체 감지 한계_좌
        self.RIGHT_OBJECT_MASK_LIMIT = 0.80             #물체 감지 한계_우

        self.TURN_RANGE = 65                            #오프셋 턴 기준값

        self.LNDcounter = 0



    def serial_write(self, num):                                #시리얼 출력 함수
        if self.ser_enabled:                                        #시리얼 사용 가능하면
            if num < 0 or num > 255:                                    #출력값이 0미만 또는 255초과라면
                logging.error("SerialError: number out of range (0-255)")   #에러레벨로 로그 출력
                return                                                      #함수 종료

            try:
                self.ser.write(bytearray([255, 85, num, 255-num, 0, 255]))  #CM530 프로토콜에 맞춰 시리얼 출력
            except serial.SerialException as e:                         #시리얼 에러 발생 시
                logging.error(f"SerialError: {str(e)}")                     #에러레벨로 로그 출력



    def vsl_rect(self, dst, rect, *args):                                               #사각형 시각화
        cv2.rectangle(dst, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), *args)   #사각형 출력 (*args : (BGR), 두께...)



    def get_theta(self, pos1, pos2):                                                            #세타 계산
        return -np.degrees(np.arctan((pos1[0]-pos2[0]) / (pos1[1]-pos2[1]))), abs(pos1[0]-pos2[0])  #로봇 추정 위치와 목표 좌표를 탄젠트 역함수로 계산한 세타 리턴



    def get_offset(self, theta, line_w, offset_limit=75): #오프셋 계산 (오프셋 한계 기본값 =75)
        offset = int(theta * (0.8 + (line_w / (self.cam_w/2))*1.75 -0.875))
        #offset = int(theta * abs(theta / 30))
        logging.debug(theta)
        logging.debug(0.8 + (line_w / (self.cam_w/2))*1.75 -0.875)
        logging.debug(offset)
        if offset <= -offset_limit:                 #오프셋이 음수 오프셋 한계보다 작으면
            return 0                                    #오프셋으로 0 리턴
        if offset >= offset_limit:                  #오프셋이 오프셋 한계의 2배보다 크다면
            return offset_limit*2                       #오프셋으로 오프셋 한계 *2 리턴
        return offset + offset_limit                #기본적으로는 오프셋 + 오프셋 한계 리턴



    def main(self): #메인 함수
        cam_ret, dst = self.cap.read()                                                                          #카메라 사용 여부와 카메라 화면 추출
        if not cam_ret:                                                                                         #카메라가 비활성화 되어있다면
            logging.warning("CameraError: Failed to capture camera")                                                #경고레벨로 로그 출력
            return False                                                                                            #메인 함수 종료 리턴

        dst = cv2.GaussianBlur(dst, (0, 0), 10)                                                                 #기본 화면에 강도 10의 블러 효과
        hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)                                                              #기본 화면의 BGR 색공간을 HSV 색공간으로 변환
        self.cam_h, self.cam_w = dst.shape[:2]                                                                  #카메라 화면의 높이와 너비 추출

        line_mask = cv2.inRange(hsv, self.MIN_LINE_HSV, self.MAX_LINE_HSV)                                      #라인 HSV 범위로 HSV 화면 이진화
        object_mask = cv2.inRange(hsv, self.MIN_OBJECT_HSV, self.MAX_OBJECT_HSV)                                #물체 HSV 범위로 HSV 화면 이진화

        line_mask[:int(self.cam_h*self.UPPER_LINE_MASK_LIMIT), :] = 0                                           #라인 감지 한계 위쪽 값 버림
        line_mask[int(self.cam_h*self.LOWER_LINE_MASK_LIMIT):, :] = 0                                           #라인 감지 한계 아래쪽 값 버림
        line_mask[:, :int(self.cam_w*self.left_line_mask_limit)] = 0                                            #라인 감지 한계 위쪽 값 버림
        line_mask[:, int(self.cam_w*(1-self.right_line_mask_limit)):] = 0                                       #라인 감지 한계 아래쪽 값 버림
        cv2.rectangle(dst, (int(self.cam_w*self.left_line_mask_limit), int(self.cam_h*self.UPPER_LINE_MASK_LIMIT)), (int(self.cam_w*(1-self.right_line_mask_limit)), int(self.cam_h*self.LOWER_LINE_MASK_LIMIT)), (127, 255, 255), 2)

        object_mask[:int(self.cam_h*self.UPPER_OBJECT_MASK_LIMIT), :] = 0                                       #물체 감지 한계 위쪽 값 버림
        object_mask[int(self.cam_h*self.LOWER_OBJECT_MASK_LIMIT):, :] = 0                                       #물체 감지 한계 아래쪽 값 버림
        object_mask[:, :int(self.cam_w*self.LEFT_OBJECT_MASK_LIMIT)] = 0                                        #물체 감지 한계 왼쪽 값 버림
        object_mask[:, int(self.cam_w*self.RIGHT_OBJECT_MASK_LIMIT):] = 0                                       #물체 감지 한계 오른쪽 값 버림
        cv2.rectangle(dst, (int(self.cam_w*self.LEFT_OBJECT_MASK_LIMIT), int(self.cam_h*self.UPPER_OBJECT_MASK_LIMIT)), (int(self.cam_w*self.RIGHT_OBJECT_MASK_LIMIT), int(self.cam_h*self.LOWER_OBJECT_MASK_LIMIT)), (255, 127, 63), 2)

        line_hulls = tr.extract_convexHulls(line_mask, min_area=self.MIN_DETECT_AREA)                           #최소 넓이를 만족하는 라인 컨벡스헐 츄출
        # line_hulls = tr.extract_approxPolyDP(line_mask, 0.005, min_area=self.MIN_DETECT_AREA)                 #라인 컨벡스헐을 0.5%의 정확도로 단순화한다
        line_hulls = tr.sort_contour_area(line_hulls)                                                           #라인 컨투어 넓이순으로 정렬
        object_rects = tr.extract_rects(object_mask, min_w=self.MIN_DETECT_WIDTH, min_h=self.MIN_DETECT_HEIGHT) #최소 너비,높이를 만족하는 물체 컨벡스헐 추출
        object_rects = tr.sort_rect_distance_point(object_rects, (self.cam_w/2, None))                          #물체 사각형의 화면 중앙에 가장 가까운 순으로 정렬


        # Object detected
        if object_rects:                                                #물체가 감지되었으면
            object_rect = object_rects[0]                                   #가장 중앙에 가까운 사각형 추출
            self.vsl_rect(dst, object_rect, (255, 127, 0), 3)               #물체 사각형 시각화
            self.serial_write(255)                                          #시리얼로 255 출력
            logging.info("Object detected    Serial written: 255(PUNCH)")   #인포레벨로 로그 출력

        # Line detected
        elif line_hulls:                                                                                            #라인이 감지되었으면
            line_hull = line_hulls[0]                                                                                   #가장 넓은 라인 컨벡스헐 선택

            cv2.drawContours(dst, line_hulls, -1, (0, 255, 255), 1)                                                     #모든 컨투어 시각화
            cv2.drawContours(dst, [line_hull], -1, (0, 255, 255), 3)                                                    #선택한 컨투어 차별화

            line_pos = tr.sel_contour_distance_point(line_hull, (self.cam_w/2, self.cam_h *0), 1)                       #라인 컨투어에서 추적할 좌표 계산
            cv2.circle(dst, line_pos, 5, (0, 0, 255), 2)
            line_pos2 = tr.sel_contour_distance_point(line_hull, (self.cam_w/2, self.cam_h * self.CURRENT_LINE_POS), 2) #라인 컨투어에서 추적할 좌표2 계산
            cv2.circle(dst, line_pos2, 5, (255, 0, 0), 2)
            line_pos = (int((line_pos[0]*2 + line_pos2[0]) /3), int((line_pos[1]*2 + line_pos2[1]) /3))

            cv2.line(dst, (int(self.cam_w/2), int(self.cam_h*self.CURRENT_LINE_POS)), line_pos, (63, 255, 127), 2)      #추적할 선 시각화
            cv2.circle(dst, line_pos, 3, (63, 255, 127), -1)                                                            #추적할 좌표 시각화

            theta, line_w = self.get_theta((self.cam_w/2, self.cam_h*self.CURRENT_LINE_POS), line_pos)                  #추적할 선과 화면 중앙 수직선으로 세타 계산


            if(line_pos[0] > self.cam_w/2):
                self.left_line_mask_limit = abs(line_pos[0] - self.cam_w/2) /self.cam_w
                self.right_line_mask_limit = 0

            if(line_pos[0] < self.cam_w/2):
                self.right_line_mask_limit = abs(line_pos[0] - self.cam_w/2) /self.cam_w
                self.left_line_mask_limit = 0


            if theta < -self.TURN_RANGE:                                            #세타가 음수 턴 기준보다 작으면
                self.serial_write(251)                                                  #시리얼로 251 출력
                logging.info(f"Theta: {theta:6.2f}    Serial written: 251(LEFT_TURN)")  #인포 레벨로 로그 출력
            elif theta > self.TURN_RANGE:                                           #세타가 양수 턴 기준보다 작으면
                self.serial_write(252)                                                  #시리얼로 252 출력
                logging.info(f"Theta: {theta:6.2f}    Serial written: 252(RIGHT_TURN)") #인포 레벨로 로그 출력
            else:                                                                   #세타가 두 기준 사이면
                offset = self.get_offset(theta, line_w)                                 #세타로 오프셋 계산
                logging.debug(offset)
                self.serial_write(offset)                                               #시리얼로 오프셋 출력
                logging.info(f"Theta: {theta:6.2f}    Serial written: {offset:3}")      #인포 레벨로 로그 출력


        # Line not detected
        else:                                                       #라인과 물체 모두 감지되지 않았다면
            self.LNDcounter += 1
            if self.LNDcounter == 15:
                logging.info(f"Line not detected({self.LNDcounter})")   #인포레벨로 로그 출력
                self.LNDcounter = 0

        cv2.imshow("dst", dst)                                      #기본 입력 화면 출력
        # cv2.imshow("line", line_mask)                             #라인 감지 화면 출력
        # cv2.imshow("object", object_mask)                         #물체 감지 화면 출력

        key = cv2.waitKey(1)                                        #1틱마다 누른 키 감지
        if key == ord("q"):                                         #"Q"키를 눌렀다면
            logging.info("Pressed 'Q'")                                 #인포레벨로 로그 출력
            return False                                                #메인 함수 종료 리턴

        return True                                                 #메인 함수 반복 리턴



    def run(self): #프로그램 실행 함수
        try:
            while True:                                 #무조건 반복
                if not self.main():                         #실행한 메인 함수에서 종료를 리턴했다면
                    break                                       #반복 종료
        except KeyboardInterrupt:                       #Ctrl C 입력 시
            logging.info("Pressed 'Ctrl+C'")                #인포레벨로 로그 출력
        finally:                                        #로그 출력 후
            self.cap.release()                          #카메라 연결 해제
            cv2.destroyAllWindows()                     #모든 창 종료
            if self.ser_enabled:                        #시리얼 사용 가능하면
                self.ser.close()                            #시리얼 통신 종료



if __name__ == "__main__":  #파일이 직접 실행되었다면
    hubot = Hubot()             #휴봇 인스턴스를 만든다
    # key = input("Press Enter to Start")
    hubot.run()                 #프로그램 실행 함수 실행