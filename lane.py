import cv2
import numpy as np
import logging
import tracker as tr
import param as p

 
logging.basicConfig(level=logging.INFO, format="[%(asctime)s] [%(levelname)s %(lineno)d] %(message)s", datefmt="%H:%M:%S")

def get_theta(pos1, pos2):                                                            #세타 계산
        if pos1 is None or pos2 is None:
            return 0,0
        return -np.degrees(np.arctan((pos1[0]-pos2[0]) / (pos1[1]-pos2[1]))), abs(pos1[0]-pos2[0])  #로봇 추정 위치와 목표 좌표를 탄젠트 역함수로 계산한 세타 리턴

def overlay_contours(src ,line_hulls, idx, bgr=(255,255,255),thk=3):
    if line_hulls:
        outline = np.zeros_like(src)
        range = np.zeros_like(src)
        for line_hull in line_hulls[:2]:
            cv2.drawContours(outline, [line_hull], idx, bgr, thk)
            cv2.drawContours(range, [line_hull], idx, bgr, -1)
        src = cv2.addWeighted(src, 1.0, range, 0.3, 0)
        src = cv2.addWeighted(src, 1.0, outline, 0.5, 0)
        return vsl_mean(src,line_hulls,7,(  0,255,255),(  0,  0,255), 2)
    return src, None


def vsl_mean(src ,line_hulls, rad, bgr1=(255,255,255), bgr2=(255,255,255),thk=3):
    cam_h, cam_w = src.shape[:2]
    c=np.array([0,0])
    tmpmask = np.zeros_like(src)
    if len(line_hulls)>1:
        for line_hull in line_hulls[:2]:
            tc1=tr.sel_contour_distance_point(line_hull,(cam_w//2, cam_h *p.CURRENT_LINE))
            tc2=tr.sel_contour_distance_point(line_hull,(cam_w//2, cam_h *0))
            tc = ((tc1[0] + tc2[0]) //2, (tc1[1] + tc2[1]) //2)
            c[0]+=tc[0]//2
            c[1]+=tc[1]//2
            cv2.circle(src, tc, 1, bgr1, thk)
            cv2.circle(src, tc, rad, bgr1, thk)
            cv2.circle(tmpmask, tc, rad, bgr1, -1)
    else:
        tc1=tr.sel_contour_distance_point(line_hulls[0],(cam_w//2, cam_h *p.CURRENT_LINE))
        tc2=tr.sel_contour_distance_point(line_hulls[0],(cam_w//2, cam_h *0))
        c = ((tc1[0] + tc2[0]) //2, (tc1[1] + tc2[1]) //2)
    cv2.circle(src, c, 1, bgr2, thk)
    cv2.circle(src, c, rad, bgr2, thk)
    cv2.circle(tmpmask, c, rad, bgr2, -1)
    return (cv2.addWeighted(src, 1.0, tmpmask, 0.25, 0), c)