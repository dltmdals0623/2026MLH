import cv2
import numpy as np
import logging
import tracker as tr
import param as p

def get_theta(pos1, pos2):
        if pos1 is None or pos2 is None:
            return 0,0
        return -np.degrees(np.arctan((pos1[0]-pos2[0]) / (pos1[1]-pos2[1])))

def overlay_contours(src ,line_hulls, idx, bgr=(255,255,255),thk=3):
    if line_hulls:
        outline = np.zeros_like(src)
        range = np.zeros_like(src)
        for line_hull in line_hulls[:2]:
            cv2.drawContours(outline, [line_hull], idx, bgr, thk)
            cv2.drawContours(range, [line_hull], idx, bgr, -1)
        src = cv2.addWeighted(src, 1.0, range, 0.3, 0)
        src = cv2.addWeighted(src, 1.0, outline, 0.5, 0)
        return vsl_mean(src,line_hulls)
    return src, None


def vsl_contours(src ,line_hulls, idx, bgr=(255,255,255)):
    if line_hulls:
        for line_hull in line_hulls[:2]:
            cv2.drawContours(src, [line_hull], idx, bgr, -1)
        return vsl_mean(src,line_hulls)
    return src, None



def vsl_mean(src, line_hulls, rad=7, bgr1=(255,127,127), bgr2=(0, 0, 255), thk=1):
    if line_hulls is None or all(h is None for h in line_hulls):
        return src, None

    tmpmask = np.zeros_like(src)
    collected_centers = []

    for line_hull in line_hulls[:2]:
        if line_hull is None:
            continue

        tc = tr.cal_contour_moment_point(line_hull)
        if tc is None:
            continue

        collected_centers.append(tc)

        cv2.circle(src, tc, rad, bgr1, thk)
        cv2.circle(tmpmask, tc, rad, bgr1,  -1)

    if len(collected_centers) == 2:
        c = ((collected_centers[0][0] + collected_centers[1][0]) // 2,
             (collected_centers[0][1] + collected_centers[1][1]) // 2)
    elif len(collected_centers) == 1:
        c = collected_centers[0]
    else:
        return src, None

    cv2.circle(src, c, rad, bgr2, thk)
    cv2.circle(tmpmask, c, rad, bgr2,  -1)
    return cv2.addWeighted(src, 1.0, tmpmask, 0.5, 0), c