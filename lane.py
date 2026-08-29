import cv2
import numpy as np
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
        _, cam_w = src.shape[:2]
#        single_c = collected_centers[0]
#        if single_c[0] < cam_w // 2:
#            c = (cam_w//2+(single_c[0]*2//3), single_c[1])
#        else:
#            c = ((single_c[0]*2//3)-cam_w//2, single_c[1])
        c=collected_centers[0]
    else:
        return src, None

    cv2.circle(src, c, rad, bgr2, thk)
    cv2.circle(tmpmask, c, rad, bgr2,  -1)
    return cv2.addWeighted(src, 1.0, tmpmask, 0.5, 0), c



def sobel(src,ksize=3):
    cam_h, cam_w = src.shape[:2]

    sobel_x = cv2.Sobel(src, cv2.CV_64F, 1, 0, ksize=ksize)
    sobel_y = cv2.Sobel(src, cv2.CV_64F, 0, 1, ksize=ksize)
    sobel_mag = np.sqrt(sobel_x**2 + sobel_y**2)

    sobel_mag = np.uint8(255 * sobel_mag / np.max(sobel_mag)) if np.max(sobel_mag) > 0 else np.uint8(sobel_mag)

    sobel_mag[:int(cam_h * p.UPPER_LINE_MASK_LIMIT), :] = 0
    sobel_mag[int(cam_h * p.LOWER_LINE_MASK_LIMIT):, :] = 0
    sobel_mag[:, :int(cam_w * p.LEFT_LINE_MASK_LIMIT)] = 0
    sobel_mag[:, int(cam_w * p.RIGHT_LINE_MASK_LIMIT):] = 0

    return cv2.threshold(sobel_mag, p.SOBEL_THRESHOLD, 255, cv2.THRESH_BINARY)



def extract_lines(edge_contours, cam_h, cam_cx):
    valid_lines = []
    for c in edge_contours:
        center_pt = tr.cal_contour_moment_point(c)
        if center_pt is not None:
            valid_lines.append({
                "contour": c,
                "center": center_pt,
                "cx": center_pt[0],  
                "dist_to_mid": abs(cam_cx - center_pt[0])+cam_h - center_pt[1]
            })
    return select_main_lanes(valid_lines, cam_cx)



def select_main_lanes(valid_lines, cam_cx):
    top_2_lines = []
    if not valid_lines:
        return top_2_lines

    valid_lines.sort(key=lambda x: x["cx"])

    groups = []
    current_group = [valid_lines[0]]
    
    for i in range(1, len(valid_lines)):
        if abs(valid_lines[i]["cx"] - current_group[-1]["cx"]) < p.GROUP_X_MARGIN:
            current_group.append(valid_lines[i])
        else:
            groups.append(current_group)
            current_group = [valid_lines[i]]
    groups.append(current_group)

    left_groups = []
    right_groups = []

    for g in groups:
        avg_cx = sum(line["cx"] for line in g) / len(g)
        if avg_cx < cam_cx:
            left_groups.append(g)
        else:
            right_groups.append(g)

    if left_groups:
        closest_left_group = min(left_groups, key=lambda g: abs((sum(l["cx"] for l in g)/len(g)) - cam_cx))
        inner_left_edge = min(closest_left_group, key=lambda l: l["dist_to_mid"])
        top_2_lines.append(inner_left_edge)

    if right_groups:
        closest_right_group = min(right_groups, key=lambda g: abs((sum(l["cx"] for l in g)/len(g)) - cam_cx))
        inner_right_edge = min(closest_right_group, key=lambda l: l["dist_to_mid"])
        top_2_lines.append(inner_right_edge)

    return [item["contour"] for item in top_2_lines]
