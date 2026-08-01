import numpy as np
import cv2

from .calc_utils import *


# contour extract functions
def extract_contours(mask, min_area=0):                                            #컨투어 검출 함수
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    #컨투어 검출
    return [c for c in contours if cv2.contourArea(c) >= min_area]                      #최소 넓이보다 넓은 컨투어 리스트 리턴



def extract_convexHulls(mask, min_area=0):                      #컨벡스헐 검출 함수
    contours = extract_contours(mask, min_area)                        #컨투어 검출
    hulls = map(lambda c: cv2.convexHull(c, clockwise=True), contours)  #컨투어에서 컨벡스헐 검출 [map(매핑 형식, 매핑 대상)]
    return list(hulls)                                                  #컨벡스헐 리스트 리턴



def extract_approxPolyDP(mask, epsilon=0.005, min_area=0):                                       #컨투어 단순화 함수
    contours = extract_contours(mask, min_area)                                                     #컨투어 검출 
    polyDPs = map(lambda c: cv2.approxPolyDP(c, epsilon*cv2.arcLength(c, True), True), contours)    #(컨투어의 닫힌 윤곽선 길이*근사치)로 컨투어 단순화
    return list(polyDPs)                                                                            #단순화한 컨투어 리스트 리턴



# contour sort function
def sort_contour_area(contours):                            #컨투어 넓이순 정렬 함수
    return sorted(contours, key=cv2.contourArea, reverse=True)  #넓이 내림차순으로 정렬된 컨투어 리스트 리턴



# contour select function
def sel_contour_distance_point(contour, pos, num=2):                            #컨투어 꼭짓점 선택 함수                                  ※np.array() 생략
    pos = sorted(contour[:, 0, :], key=lambda p: distance(p, pos), reverse=True)    #컨투어의 꼭짓점을 기준점에서 가장 먼 순으로 정렬        ex) conture =[ [[0,0]], [[0,1]], [[0,2]] ] ->contour[:, 0, :] =[ [0,0], [0,1], [0,2] ]
    return tuple(map(int, np.average(pos[:num], axis=0)))                           #앞쪽 좌표 num개의 x, y좌표끼리 평균을 계산한 값 리턴    ex)[[50,60],[70,80]] -> (60,70)


    
def cal_contour_moment_point(contour):
    M = cv2.moments(contour)
    if M["m00"] != 0:
        return (int(M["m10"] / M["m00"]),int(M["m01"] / M["m00"]))