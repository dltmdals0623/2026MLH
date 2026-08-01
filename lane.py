import cv2
import numpy as np
import tracker as tr
import param as p



def get_theta(pos1, pos2):
        if pos1 is None or pos2 is None:
            return 0,0
        return -np.degrees(np.arctan((pos1[0]-pos2[0]) / (pos1[1]-pos2[1]))), abs(pos1[0]-pos2[0])



def overlay_contours(src ,line_hulls, idx, bgr=(255,255,255),thk=3):
    if line_hulls:
        outline = np.zeros_like(src)
        range = np.zeros_like(src)
        for line_hull in line_hulls[:2]:
            cv2.drawContours(outline, [line_hull], idx, bgr, thk)
            cv2.drawContours(range, [line_hull], idx, bgr, -1)
        src = cv2.addWeighted(src, 1.0, range, 0.3, 0)
        src = cv2.addWeighted(src, 1.0, outline, 0.5, 0)
    return src



def vsl_line(src,line_hulls,idx,colors=[(0, 255, 0), (0, 0, 255)],thk=3):
    if all(i is None for i in line_hulls):
        return src, None
    Rline=[]
    for i in range(len(line_hulls)):
        if line_hulls[i] is not None:
            Rline.append(line_hulls[i])
            src = overlay_contours(src, [line_hulls[i]], idx, colors[i], thk)
    # print('\t\t',len(Rline))
    return [src, Rline]



def vsl_mean(src, edges, rad=6, bgr1=(0, 255, 0), bgr2=(0, 0, 255), target_bgr=(0, 255, 255), thk=3):
    if not edges or all(e is None for e in edges):
        return src, None

    cam_h, cam_w = src.shape[:2]
    tmpmask = np.zeros_like(src)
    collected_centers = []
    
    # 좌, 우 라인 각각의 색상을 다르게 주기 위한 리스트 (bgr1: 좌측, bgr2: 우측)
    colors = [bgr1, bgr2]

    for idx, edge_contour in enumerate(edges):
        if edge_contour is None:
            continue

        # 1. 상하 직선성 강조를 위한 다중 샘플링 (꺾임 및 노이즈 방지)
        vertical_points = []
        steps = 5
        for i in range(steps):
            y_ratio = 0.2 + (0.6 * i / (steps - 1)) # 화면 높이 20% ~ 80% 구간
            target_y = int(cam_h * y_ratio)
            
            # contour 내에서 target_y와 가장 가까운 점 추출
            pt = tr.sel_contour_distance_point(edge_contour, (cam_w // 2, target_y))
            if pt is not None:
                vertical_points.append(pt)

        # 2. 대표 중앙점 계산
        if vertical_points:
            xs = [p[0] for p in vertical_points]
            ys = [p[1] for p in vertical_points]
            # 꺾이는 대각선 끝단에 쏠리지 않도록 X값은 중앙값(Median) 사용
            mean_x = int(np.median(xs))
            mean_y = int(np.mean(ys))
            tc = (mean_x, mean_y)
        else:
            # 다중 샘플링 실패 시 기존 모멘트 중심점으로 Fallback
            fallback_pt = tr.cal_contour_moment_point(edge_contour)
            if fallback_pt is not None:
                tc = fallback_pt
            else:
                continue

        collected_centers.append(tc)
        color = colors[idx % len(colors)]

        # 3. 개별 라인 및 중심점 시각화
        cv2.drawContours(src, [edge_contour], -1, color, thk + 1)
        cv2.circle(src, tc, rad, (255, 0, 255), -1) # 핑크색 중심점

    # 4. 최종 중심점(c) 계산
    c = None
    if len(collected_centers) > 1:
        # 양쪽 라인이 모두 있을 경우 중앙값
        c_x = (collected_centers[0][0] + collected_centers[1][0]) // 2
        c_y = (collected_centers[0][1] + collected_centers[1][1]) // 2
        c = (c_x, c_y)
    elif len(collected_centers) == 1:
        # 한쪽만 있을 경우 해당 라인의 중심
        c = collected_centers[0]

    # 5. 최종 타겟 중심점 및 조향 가이드라인 시각화
    if c is not None:
        cv2.circle(src, c, rad + 2, target_bgr, -1) # 노란색 타겟점
        cv2.line(tmpmask, (cam_w // 2, int(cam_h * p.CURRENT_LINE)), c, (0, 0, 255), 2)
        src = cv2.addWeighted(src, 1.0, tmpmask, 0.3, 0)

    return src, c



def extract_sobel_lane_edges(line_mask, min_lane_area=1000, min_edge_area=50):
    """
    line_mask에서 중앙(+-100px)에 위치한 가장 큰 래인을 찾고,
    Sobel 엣지를 적용하여 좌/우 메인 엣지를 추출하는 함수
    """
    cam_h, cam_w = line_mask.shape[:2]
    cam_center_x = cam_w // 2

    # 1. 전체 마스크에서 컨투어 검출 (최소 넓이 이상)
    lane_contours = tr.extract_contours(line_mask, min_lane_area)
    if not lane_contours:
        return None, None

    # =====================================================================
    # [수정된 부분] 화면 중앙 기준 +- 100px 이내에 있는 컨투어만 필터링
    # =====================================================================
    valid_lanes = []
    for c in lane_contours:
        center_pt = tr.cal_contour_moment_point(c)
        if center_pt is not None:
            cx, cy = center_pt
            # X 좌표가 중앙에서 -100 ~ +100 범위 안에 있는지 확인
            if (cam_center_x - 100) <= cx <= (cam_center_x + 100):
                valid_lanes.append(c)
                
    # 중앙 +- 100px 이내에 유효한 라인이 전혀 없다면 탐지 실패로 간주
    if not valid_lanes:
        return None, None

    # 범위 내에 들어온 유효한 라인들 중 가장 넓이가 큰 것을 메인 래인으로 선별
    sorted_lane_contours = tr.sort_contour_area(valid_lanes)
    main_lane = sorted_lane_contours[0]
    # =====================================================================

    # 2. 메인 래인 전용 캔버스 생성
    lane_canvas = np.zeros_like(line_mask)
    cv2.drawContours(lane_canvas, [main_lane], -1, 255, -1)

    # 3. Sobel 엣지 추출
    sobel_x = cv2.Sobel(lane_canvas, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(lane_canvas, cv2.CV_64F, 0, 1, ksize=3)
    sobel_mag = np.sqrt(sobel_x**2 + sobel_y**2)

    sobel_mag = np.uint8(255 * sobel_mag / np.max(sobel_mag)) if np.max(sobel_mag) > 0 else np.uint8(sobel_mag)
    threshold_val = getattr(p, 'SOBEL_THRESHOLD', 50) 
    _, sobel_edge = cv2.threshold(sobel_mag, threshold_val, 255, cv2.THRESH_BINARY)

    sobel_edge[:int(cam_h * p.UPPER_LINE_MASK_LIMIT+1), :] = 0
    sobel_edge[int(cam_h * p.LOWER_LINE_MASK_LIMIT-1):, :] = 0
    sobel_edge[:, :int(cam_w * p.LEFT_LINE_MASK_LIMIT+1)] = 0
    sobel_edge[:, int(cam_w * p.RIGHT_LINE_MASK_LIMIT-1):] = 0

    # 5. 추출된 단일 래인의 엣지에서 다시 컨투어 검출
    edge_contours = tr.extract_contours(sobel_edge, min_edge_area)

    left_edges, right_edges = [], []
    for c in edge_contours:
        center_pt = tr.cal_contour_moment_point(c)
        if center_pt is not None:
            if center_pt[0] < cam_center_x:
                left_edges.append(c)
            else:
                right_edges.append(c)

    # 6. 좌우 각각 가장 큰(긴) 엣지 컨투어 선별
    left_edge = tr.sort_contour_area(left_edges)[0] if left_edges else None
    right_edge = tr.sort_contour_area(right_edges)[0] if right_edges else None

    return left_edge, right_edge



def vsl_sobel_lanes(dst, line_mask):
    cam_h, cam_w = dst.shape[:2]
    cam_center_x = cam_w // 2

    # 1. 화면 중앙 세로 가이드라인 시각화
    cv2.line(dst, (cam_center_x, 0), (cam_center_x, cam_h), (0, 255, 255), 1)

    # 2. 앞서 만들어둔 함수로 좌/우 엣지 컨투어 추출
    left_edge, right_edge = extract_sobel_lane_edges(line_mask, min_lane_area=1000, min_edge_area=50)
    edges = [left_edge, right_edge]

    # 3. vsl_mean에 시각화 및 타겟 포인트 계산 위임
    dst, c = vsl_mean(dst, edges, rad=6, bgr1=(0, 255, 0), bgr2=(0, 0, 255), target_bgr=(0, 255, 255), thk=3)

    return dst, c