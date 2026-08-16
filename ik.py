import numpy as np
import param as p

def calculate_ik_3dof(x, y, z):
    try:
        R_yz = np.sqrt(y**2 + z**2)
        if R_yz < p.L1:
            raise ValueError("기저 반경보다 가까움")

        z1 = -np.sqrt(R_yz**2 - p.L1**2)
        phi = np.atan2(z1, p.L1)
        psi = np.atan2(z, y)
        theta_1 = psi - phi

        x1 = x

        original_D = np.sqrt(x1**2 + z1**2)
        
        max_reach = p.L2 + p.L3
        min_reach = abs(p.L2 - p.L3)

        if original_D-max_reach > 1:
            return None
        elif original_D > max_reach:
            D = max_reach
        elif original_D < min_reach:
            D = min_reach
        else:
            D = original_D

        cos_theta_3 = (D**2 - p.L2**2 - p.L3**2) / (2 * p.L2 * p.L3)
        cos_theta_3 = max(-1.0, min(1.0, cos_theta_3))
        theta_3 = np.acos(cos_theta_3)

        theta_2 = np.atan2(z1, x1) - np.atan2(p.L3 * np.sin(theta_3), p.L2 + p.L3 * np.cos(theta_3))

        th1 = theta_1
        th2 = theta_2
        th3 = theta_3

        p0 = (0, 0, 0)
        
        x1_j1 = 0
        z1_j1 = 0
        y1_j1 = p.L1
        p1 = (
            x1_j1,
            y1_j1 * np.cos(th1) - z1_j1 * np.sin(th1),
            y1_j1 * np.sin(th1) + z1_j1 * np.cos(th1))

        x1_j2 = p.L2 * np.sin(th2)
        z1_j2 = -(p.L2 * np.cos(th2))
        y1_j2 = p.L1
        p2 = (
            x1_j2,
            y1_j2 * np.cos(th1) - z1_j2 * np.sin(th1),
            y1_j2 * np.sin(th1) + z1_j2 * np.cos(th1))

        x1_j3 = p.L2 * np.sin(th2) + p.L3 * np.sin(th2 - th3)
        z1_j3 = -(p.L2 * np.cos(th2) + p.L3 * np.cos(th2 - th3))
        y1_j3 = p.L1
        p3 = (
            x1_j3,
            y1_j3 * np.cos(th1) - z1_j3 * np.sin(th1),
            y1_j3 * np.sin(th1) + z1_j3 * np.cos(th1))

        theta_2 = np.atan2(z1, x1) - np.atan2(p.L3 * np.sin(theta_3), p.L2 + p.L3 * np.cos(theta_3))

        return [theta_1, theta_2, theta_3] 
        
    except Exception:
        return None

def cal_leg_offset(delta_xyz, inv=0):
    if delta_xyz == [0,0,0]:
        return [512,512,512]

    target_x = 9 + (delta_xyz[0] * 1000)
    target_y = 27 + (delta_xyz[1] * 1000)
    target_z = -108 + (delta_xyz[2] * 1000)

    ik_res = calculate_ik_3dof(target_x, target_y, target_z)

    if ik_res is None:
        return [512, 512, 512]
    
    # [수정됨] ik_res[1][0]이 아닌, ik_res 배열의 각도를 순서대로 꺼냅니다.
    offset1 = (lambda th: (512 - int((np.degrees(th) + 180.0) * (1024 / 360))))(ik_res[0])
    offset2 = (lambda th: (512 - int((np.degrees(th) + 180.0) * (1024 / 360))))(ik_res[1])
    offset3 = (lambda th: (512 - int((np.degrees(th) + 180.0) * (1024 / 360))))(ik_res[2])

    # 만약 좌/우측 다리의 모터 방향이 반대(inv)라면 1024에서 빼주는 등의 추가 처리가 필요할 수 있습니다.

    return [offset1, offset2, offset3]



LEG_POSITIONS = {
    "Front-Left":  ( 0.12,  0.08),  # 모터 ID: 1, 2, 3 (그룹에 따라 다름)
    "Front-Right": ( 0.12, -0.08),  # 모터 ID: 4, 5, 6 ...
    "Rear-Left":   (-0.12,  0.08),
    "Rear-Right":  (-0.12, -0.08)
}

def calculate_cmd_vel_offsets(v_x, v_y, w_z, stance_time=1.0):
    """
    [요구사항 구현] 선속도(V_x, V_y)와 각속도(W_z)를 입력받아 
    4개 다리 각각의 Stance Phase 동안 이동해야 할 목표 X, Y 오프셋을 계산합니다.
    """
    leg_offsets = {}
    
    for leg_name, (x, y) in LEG_POSITIONS.items():
        # [Step 1] 선속도 성분 (Linear Velocity Component)
        # 로봇이 전진/좌진할 때 딛고 있는 다리는 반대 방향으로 밀어내야 하므로 부호 반전(-)
        v_leg_x_linear = -v_x
        v_leg_y_linear = -v_y
        
        # [Step 2] 각속도에 의한 회전 성분 (Angular Velocity Component)
        # 수식: V_leg_x = Y * W_z, V_leg_y = -X * W_z
        v_leg_x_angular =  y * w_z
        v_leg_y_angular = -x * w_z
        
        # [Step 3] 성분 합성 (Total Leg Velocity)
        v_leg_x = v_leg_x_linear + v_leg_x_angular
        v_leg_y = v_leg_y_linear + v_leg_y_angular
        
        # [Step 4] 속도에 시간을 곱해 최종 위치 오프셋(delta_X, delta_Y) 산출
        delta_x = v_leg_x * stance_time
        delta_y = v_leg_y * stance_time
        
        leg_offsets[leg_name] = [delta_x, delta_y, 0.0] # Z축 오프셋은 기본 0으로 설정
        
    return leg_offsets

def cal_offset_with_cmd_vel(motor_ids, v_x, v_y, w_z, stance_time=1.0):
    """
    cmd_vel 기반으로 계산된 다리별 오프셋을 기존 역운동학(calculate_ik_3dof)과 
    모터 각도 변환 함수(cal_leg_offset)에 매핑하여 패킷 데이터 형태로 반환합니다.
    """
    if v_x == 0.0 and v_y == 0.0 and w_z == 0.0:
        return [(i, 512) for i in motor_ids]
        
    # 4개 다리의 공간상 목표 오프셋 계산
    offsets_xyz = calculate_cmd_vel_offsets(v_x, v_y, w_z, stance_time)
    
    # 1. 각 다리별 역운동학(관절 각도 오프셋) 계산
    leg_ik_results = {
        "Front-Left":  cal_leg_offset(offsets_xyz["Front-Left"], inv=0),
        "Front-Right": cal_leg_offset(offsets_xyz["Front-Right"], inv=1),
        "Rear-Left":   cal_leg_offset(offsets_xyz["Rear-Left"], inv=0),
        "Rear-Right":  cal_leg_offset(offsets_xyz["Rear-Right"], inv=1)
    }
    
    # 2. 첨부된 하드웨어 3D 렌더링 기준 모터 ID 명시적 맵핑
    # 튜플 구성: (Coxa_ID, Femur_ID, Tibia_ID) 순서
    motor_mapping = {
        "Front-Right": (1, 3, 5),   # 우측 앞
        "Front-Left":  (2, 4, 6),   # 좌측 앞
        "Rear-Right":  (7, 9, 11),  # 우측 뒤
        "Rear-Left":   (8, 10, 12)  # 좌측 뒤
    }
    
    # 3. 모터 ID별 패킷 포맷 (ID, Value)으로 변환 패킹
    offsets = []
    
    for leg_name, ids in motor_mapping.items():
        res = leg_ik_results[leg_name]
        
        # IK 연산이 성공적으로 수행되어 반환값이 존재할 경우에만
        if res: 
            # 각 다리에는 3개의 관절(0:Coxa, 1:Femur, 2:Tibia)이 있으므로 3번 반복
            for j in range(3): 
                # 활성화된 motor_ids 리스트에 해당 ID가 포함되어 있다면 오프셋 추가
                if ids[j] in motor_ids:
                    offsets.append((ids[j], res[j]))
                    
    return offsets