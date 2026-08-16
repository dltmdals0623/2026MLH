import numpy as np
import cv2

from .calc_utils import *


# rectangle extract function
def extract_rects(mask, min_area=0, min_w=0, min_h=0):                              #사각형 검출 함수
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    #컨투어 검출
    contours = filter(lambda c: cv2.contourArea(c)>=min_area, contours)                 #최소 넓이 조건으로 컨투어 리스트 필터링

    rects = map(lambda c: cv2.boundingRect(c), contours)                                #컨투어에서 사각형 검출
    rects = filter(lambda r: r[2]>=min_w and r[3]>=min_h, rects)                        #최소 너비, 높이로 사각형 리스트 필터링
    return list(rects)                                                                  #필터링된 사각형 리스트 리턴



# rectangle sort functions
def sort_rect_distance_point(rects, pos):                           #사각형 꼭짓점 거리순 선택 함수
    return sorted(rects, key=lambda r: distance(pos, rect_center(r)))   #중앙 좌표와 기준점의 거리 오름차순으로 정렬된 사각형 리스트 리턴



def sort_rect_distance_stack(rects, stack, weights=None):   #사각형 스택 거리순 정렬 함수
    if not stack:                                               #rect가 스택이 아니라면
        return rects                                                #rect 그대로 리턴

    if weights is not None:                 #가중치가 존재하면
        min_len = min(len(stack), len(weights)) #스택과 가중치 리스트 중 더 짧은 리스트 길이로 최소 길이 설정
        weights = weights[-min_len:]            #가중치 리스트 최소 길이 맞춤
        stack = stack[-min_len:]                #스택 최소 길이 맞춤

    return sorted(rects, key=lambda r: np.average(                          #아래의 평균값으로 정렬한 사각형 리스트 리턴
                  [distance(rect_center(s), rect_center(r)) for s in stack],    #스택의 사각형 중앙 좌표와 rects의 사각형 중앙 좌표의 거리 리스트 생성
                  weights=weights))                                             #가중치 사용



def sort_rect_size(rects):                                  #사각형 넓이순 정렬 함수
    return sorted(rects, key=lambda r: r[2]*r[3], reverse=True) #넓이 내림차순으로 정렬된 컨투어 리스트 리턴



# point select functions
def sel_rect_center(rect):                  #사각형 중앙 좌표 선택
    return tuple(map(int, rect_center(rect)))   #사각형 중앙 좌표 정수로 리턴 (rect_center() 출력은 float)



def sel_rect_distance_corner(rect, pos):    #사각형 {}점 선택 함수
    x, y, w, h = rect   #사각형 구성요소 추출

    x_valid = pos[0] is not None    #pos의 x좌표 존재 여부
    y_valid = pos[1] is not None    #pos의 y좌표 존재 여부

    if x_valid and y_valid:                                                                 #x, y좌표가 모두 존재하면
        return max([(x, y), (x+w, y), (x, y+h), (x+w, y+h)], key=lambda p: distance(pos, p))    #사각형의 네 꼭짓점 중 기준에서 가장 먼 꼭짓점 리턴
    if x_valid:                                                                             #x좌표만 존재하면
        return (max([x, x+w], key=lambda p: abs(pos[0]-p)), y+h//2)                             #사각형의 좌우 두변 중 입력받은 좌표에서 가장 먼 변의 중점 리턴
    if y_valid:                                                                             #y좌표만 존재하면
        return (x+w//2, max([y, y+h], key=lambda p: abs(pos[1]-p)))                             #사각형의 상하 두변 중 입력받은 좌표에서 가장 먼 변의 중점 리턴
    print("\033[32mSortError: Cannot calculate distance\033[0m")                            #x, y좌표가 모두 없으면 에러레벨로 로그 출력
    return None                                                                             #빈 값 리턴



def sel_rect_average_point_power(rect, pos, max_size, power=2): #사각형 가중치 거리 계산 함수
    _, _, w, _ = rect                                               #사각형 너비 추출
    cx, cy = sel_rect_center(rect)                                      #cx, cy에 사각형의 중앙 좌표 대입 (center)
    dx, dy = sel_rect_distance_corner(rect, pos)                        #dx, dy에 사각형의 기준점에서 가장 먼 {}점 대입 (distance)

    x_valid = pos[0] is not None and max_size[0] is not None    #pos1과 pos2의 x좌표 존재 여부
    y_valid = pos[1] is not None and max_size[1] is not None    #pos1과 pos2의 y좌표 존재 여부

    if x_valid and y_valid:                                                                         #x, y좌표가 모두 존재하면
        return tuple(map(int,                                                                           #아래의 평균 좌표를 정수로 리턴
                (np.average([cx, dx], weights=[1-(w/max_size[0])**power,(w/max_size[0])**power]),           #(cx와 dx의 평균 계산 {사각형의 너비가 클수록 dx에, 작을수록 cx에 가중치 ^ power},
                 np.average([cy, dy], weights=[-(w/max_size[1])**power, (w/max_size[1])**power]))))         #cy와 dy의 평균 계산 {사각형의 너비가 클수록 dy에, 작을수록 cy에 가중치 ^ power})
    if x_valid:                                                                                     #x좌표만 존재하면
        return tuple(map(int,                                                                           #아래의 평균 좌표를 정수로 리턴
                (np.average([cx, dx], weights=[1-(w/max_size[0])**power, (w/max_size[0])**power]), cy)))    #(cx와 dx의 평균 {사각형의 너비가 클수록 dx에, 작을수록 cx에 가중치 ^ power}, cy)
    if y_valid:                                                                                     #y좌표만 존재하면
        return tuple(map(int,                                                                           #아래의 평균 좌표를 정수로 리턴
                (cx, np.average([cy, dy], weights=[1-(w/max_size[1])**power, (w/max_size[1])**power]))))    #(cx, cy와 dy의 평균 {사각형의 너비가 클수록 dy에, 작을수록 cy에 가중치 ^ power})
    return None                                                                                     #x, y좌표가 모두 없으면 빈 값 리턴