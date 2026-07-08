import tkinter as tk
import math


# ==========================================
# 1. 역기구학(IK) 및 정기구학(FK) 수식 
# ==========================================
def calculate_ik_3dof(x, y, z, L1, L2, L3):
    """주어진 목표 좌표에 대한 관절 각도를 계산합니다."""
    try:
        # 1. 관절 1 계산 (X축 회전)
        R_yz = math.sqrt(y**2 + z**2)
        if R_yz < L1:
            raise ValueError("기저 반경보다 가까움")
            
        z1 = -math.sqrt(R_yz**2 - L1**2)
        phi = math.atan2(z1, L1)
        psi = math.atan2(z, y)
        theta_1 = psi - phi
        
        # 2. 로컬 X-Z 평면 매핑
        x1 = x
        
        # 원점에서 타겟까지의 2D 평면 직선 거리
        original_D = math.sqrt(x1**2 + z1**2)
        
        # [수정된 부분] 작업공간 제한 (Clamping)
        # 에러를 뱉는 대신, 길이를 최대(L2+L3) 및 최소(|L2-L3|)로 강제 고정합니다.
        max_reach = L2 + L3
        min_reach = abs(L2 - L3)
        
        if original_D-max_reach > 1:
            return None
        elif original_D > max_reach:
            D = max_reach
        elif original_D < min_reach:
            D = min_reach
        else:
            D = original_D
            
        # 3. 관절 3 계산 (무릎)
        cos_theta_3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)
        
        # D가 max_reach일 때 cos_theta_3는 정확히 1이 되어 theta_3은 0도(완전히 펴짐)가 됩니다.
        cos_theta_3 = max(-1.0, min(1.0, cos_theta_3))
        theta_3 = math.acos(cos_theta_3)
        
        # 4. 관절 2 계산 (엉덩이)
        # alpha는 제한된 D가 아닌 '원래 타겟의 방향(x1, z1)'을 유지하도록 계산합니다.
        # 이렇게 해야 다리가 완전히 펴진 상태로 타겟 쪽을 정확히 가리킵니다.
        alpha = math.atan2(x1, -z1)
        
        cos_beta = (L2**2 + D**2 - L3**2) / (2 * L2 * D)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        
        theta_2 = alpha - beta
        
        return theta_1, theta_2, theta_3
        
    except ValueError:
        return None

def get_joint_positions(th1, th2, th3, L1, L2, L3):
    """관절 각도를 바탕으로 각 조인트의 3차원 글로벌 좌표를 구합니다."""
    p0 = (0, 0, 0) # Base
    
    # Joint 1 (Link 1 끝단)
    p1 = (0, L1 * math.cos(th1), L1 * math.sin(th1))
    
    # Joint 2 (Link 2 끝단)
    x1_j2 = L2 * math.sin(th2)
    z1_j2 = -L2 * math.cos(th2)
    y1_j2 = L1
    p2 = (
        x1_j2,
        y1_j2 * math.cos(th1) - z1_j2 * math.sin(th1),
        y1_j2 * math.sin(th1) + z1_j2 * math.cos(th1)
    )
    
    # Joint 3 (Link 3 끝단 / End-effector)
    x1_j3 = L2 * math.sin(th2) + L3 * math.sin(th2 + th3)
    z1_j3 = -(L2 * math.cos(th2) + L3 * math.cos(th2 + th3))
    y1_j3 = L1
    p3 = (
        x1_j3,
        y1_j3 * math.cos(th1) - z1_j3 * math.sin(th1),
        y1_j3 * math.sin(th1) + z1_j3 * math.cos(th1)
    )
    
    return [p0, p1, p2, p3]





# ==========================================
# 2. 모터 오프셋 계산 함수
# ==========================================
def cal_joint_offset(current_joints, delta_xyz, L1=46.5, L2=38.0, L3=42.0):
    """
    현재 모터 스텝 각도에서 특정 변위만큼 이동할 때 필요한 관절 모터의 변화량(오프셋)을 계산합니다.
    
    [입력]
    - current_joints: (th1, th2, th3) 현재 관절 모터 스텝값 (0~1023) 튜플
    - delta_xyz: (dx, dy, dz) 이동하고자 하는 3차원 변위 튜플
    - L1, L2, L3: 로봇 다리 링크 길이 (기본값 설정됨)
    
    [출력]
    - (d_th1, d_th2, d_th3): 목표 위치 도달을 위한 모터 스텝 변화량 튜플
    - None: 목표 위치가 작업 공간을 벗어나 도달할 수 없는 경우
    """
    
    # --- [0] 모터 스텝값(0~1023)을 수학적 연산을 위한 라디안(Radian)으로 변환 ---
    # 기존 코드의 변환식: step = (math.degrees(rad) + 180.0) * (1024 / 360)
    # 위 식의 역연산을 수행하여 모터 스텝을 라디안으로 복원합니다.
    rad_joints = []
    for step in current_joints:
        deg = (step * (360.0 / 1024.0)) - 180.0
        rad = math.radians(deg)
        rad_joints.append(rad)
        
    current_th1, current_th2, current_th3 = rad_joints
    dx, dy, dz = delta_xyz

    # --- [1] 현재 좌표 계산 (정기구학 활용) ---
    # get_joint_positions 함수는 Base(p0), Joint1(p1), Joint2(p2), End-effector(p3) 리스트를 반환합니다.
    positions = get_joint_positions(current_th1, current_th2, current_th3, L1, L2, L3)
    current_x, current_y, current_z = positions[3]  # 말단 장치(p3)의 3차원 글로벌 좌표 추출

    # --- [2] 목표 좌표 계산 ---
    # 현재 말단 장치의 좌표에 입력받은 이동 변위를 더합니다.
    target_x = current_x + dx
    target_y = current_y + dy
    target_z = current_z + dz

    # --- [3] 목표 관절 각도 계산 (역기구학 활용) ---
    # 목표 좌표를 calculate_ik_3dof에 넣어 새로운 관절 각도를 라디안 단위로 구합니다.
    ik_result = calculate_ik_3dof(target_x, target_y, target_z, L1, L2, L3)

    # --- [4] 예외 처리 ---
    # 역기구학 계산 결과가 None이면 (작업 공간 이탈 등) 함수도 None을 반환하고 종료합니다.
    if ik_result is None:
        return None

    new_th1, new_th2, new_th3 = ik_result

    # --- [5] 오프셋 계산 및 반환 ---
    # 새롭게 구해진 라디안 각도를 다시 모터 스텝값(0~1023)으로 변환합니다.
    new_steps = []
    for rad in (new_th1, new_th2, new_th3):
        deg = math.degrees(rad) + 180.0
        step = round(deg * (1024/360))
        # 모터 스텝의 한계치(0 ~ 1023)를 벗어나지 않도록 클리핑 처리를 추가합니다.
        step = max(0, min(1023, step))
        new_steps.append(step)

    # 새로운 모터 스텝값에서 현재 모터 스텝값을 빼서 변화량(Delta)을 구합니다.
    d_th1 = new_steps[0] - current_joints[0]
    d_th2 = new_steps[1] - current_joints[1]
    d_th3 = new_steps[2] - current_joints[2]

    return (d_th1, d_th2, d_th3)





# ==========================================
# 3. Tkinter 3D GUI 시각화 클래스
# ==========================================
class RobotLegApp:
    def __init__(self, root):
        self.root = root
        self.root.title("3DOF 로봇 다리 IK 3D 시각화")
        
        self.L1, self.L2, self.L3 = 46.5, 38.0, 42.0
        self.prev_joints = None
        
        self.init_ui()
        self.update_view()

    def init_ui(self):
        # 왼쪽 컨트롤 패널 (슬라이더 및 출력)
        ctrl_frame = tk.Frame(self.root, padx=15, pady=15)
        ctrl_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        tk.Label(ctrl_frame, text="목표 좌표 (Target XYZ)", font=("Arial", 11, "bold")).pack(pady=5)
        self.scale_x = tk.Scale(ctrl_frame, from_=-100, to=100, orient=tk.HORIZONTAL, label="X 좌표", length=220, command=self.update_view)
        self.scale_x.set(0)
        self.scale_x.pack()
        
        self.scale_y = tk.Scale(ctrl_frame, from_=-20, to=150, orient=tk.HORIZONTAL, label="Y 좌표", length=220, command=self.update_view)
        self.scale_y.set(47)
        self.scale_y.pack()
        
        self.scale_z = tk.Scale(ctrl_frame, from_=-120, to=50, orient=tk.HORIZONTAL, label="Z 좌표", length=220, command=self.update_view)
        self.scale_z.set(-60)
        self.scale_z.pack()
        
        tk.Label(ctrl_frame, text="------------------------------").pack(pady=5)
        
        tk.Label(ctrl_frame, text="3D 뷰 회전 (View Angle)", font=("Arial", 11, "bold")).pack(pady=5)
        self.scale_az = tk.Scale(ctrl_frame, from_=0, to=360, orient=tk.HORIZONTAL, label="방위각", length=220, command=self.update_view)
        self.scale_az.set(135)
        self.scale_az.pack()
        
        self.scale_el = tk.Scale(ctrl_frame, from_=-90, to=90, orient=tk.HORIZONTAL, label="고도각", length=220, command=self.update_view)
        self.scale_el.set(20)
        self.scale_el.pack()
        
        # --- 추가된 부분: 모터(서보) 기준 각도 출력 ---
        tk.Label(ctrl_frame, text="------------------------------").pack(pady=5)
        tk.Label(ctrl_frame, text="관절 출력 각도 (기준 적용)", font=("Arial", 11, "bold")).pack(pady=5)
        
        self.angle1_var = tk.StringVar()
        self.angle2_var = tk.StringVar()
        self.angle3_var = tk.StringVar()
        
        # UI 가시성을 위해 다리 색상과 동일하게 설정
        tk.Label(ctrl_frame, textvariable=self.angle1_var, fg="#FF4444", font=("Arial", 11, "bold")).pack(pady=2)
        tk.Label(ctrl_frame, textvariable=self.angle2_var, fg="#44AA44", font=("Arial", 11, "bold")).pack(pady=2)
        tk.Label(ctrl_frame, textvariable=self.angle3_var, fg="#4444FF", font=("Arial", 11, "bold")).pack(pady=2)

        # 상태 표시 라벨
        self.status_var = tk.StringVar()
        self.status_label = tk.Label(ctrl_frame, textvariable=self.status_var, font=("Arial", 11, "bold"))
        self.status_label.pack(pady=20)
        
        # 오른쪽 캔버스 (그림 영역)
        self.canvas_w, self.canvas_h = 700, 800
        self.canvas = tk.Canvas(self.root, width=self.canvas_w, height=self.canvas_h, bg="#ffffff", relief="sunken", bd=2)
        self.canvas.pack(side=tk.RIGHT, padx=10, pady=10)

    def project_3d(self, x, y, z, az, el, scale=3.5):
        """3D 좌표를 2D 화면 좌표로 투영"""
        rad_az = math.radians(az)
        x_r1 = x * math.cos(rad_az) - y * math.sin(rad_az)
        y_r1 = x * math.sin(rad_az) + y * math.cos(rad_az)
        z_r1 = z
        
        rad_el = math.radians(el)
        x_r2 = x_r1
        y_r2 = y_r1 * math.cos(rad_el) - z_r1 * math.sin(rad_el)
        z_r2 = y_r1 * math.sin(rad_el) + z_r1 * math.cos(rad_el)
        
        screen_x = (self.canvas_w / 2) + x_r2 * scale
        screen_y = (self.canvas_h / 2) - z_r2 * scale
        
        depth = y_r2
        return screen_x, screen_y, depth

    def get_perspective_radius(self, depth):
        scale_factor = 1.0 - (depth / 300.0)
        return max(2.0, 10.0 * scale_factor)

    def update_view(self, *_):
        self.canvas.delete("all")
        
        tx, ty, tz = self.scale_x.get(), self.scale_y.get(), self.scale_z.get()
        az, el = self.scale_az.get(), self.scale_el.get()
        
        ik_res = calculate_ik_3dof(tx, ty, tz, self.L1, self.L2, self.L3)
        
        if ik_res is None:
            self.status_var.set("상태: 도달 불가 (Workspace 이탈)")
            self.status_label.config(fg="red")
            
            # 실패 시 각도 표기 지우기
            self.angle1_var.set("Joint 1: Error")
            self.angle2_var.set("Joint 2: Error")
            self.angle3_var.set("Joint 3: Error")
            
            joints = self.prev_joints
            color_dim = True
        else:
            self.status_var.set("상태: 정상 도달")
            self.status_label.config(fg="black")
            
            # --- 지정된 기준(오프셋)으로 각도 계산 ---
            th1, th2, th3 = ik_res
            
            # 1. 1번 관절: y축 +방향(수학적 0도)이 150도가 되도록 변환
            disp_th1 = math.degrees(th1) + 180.0
            
            # 2. 2,3번 관절: x축 -방향(수학적 -90도)이 150도가 되도록 변환 (-90 + 240 = 150)
            disp_th2 = math.degrees(th2) + 180.0
            disp_th3 = math.degrees(th3) + 180.0
            
            self.angle1_var.set(f"Joint 1 (Red): {disp_th1:.1f}°, {disp_th1*(1024/360):.0f}")
            self.angle2_var.set(f"Joint 2 (Green): {disp_th2:.1f}° {disp_th2*(1024/360):.0f}")
            self.angle3_var.set(f"Joint 3 (Blue): {disp_th3:.1f}° {disp_th3*(1024/360):.0f}")
            
            joints = get_joint_positions(*ik_res, self.L1, self.L2, self.L3)
            self.prev_joints = joints
            color_dim = False

        render_list = []
        
        axes = [((100,0,0), 'red', 'X'), ((0,100,0), 'green', 'Y'), ((0,0,100), 'blue', 'Z')]
        for pt, color, label in axes:
            sx1, sy1, d1 = self.project_3d(0, 0, 0, az, el)
            sx2, sy2, d2 = self.project_3d(*pt, az, el)
            avg_d = (d1 + d2) / 2
            render_list.append({'type': 'line', 'coords': (sx1, sy1, sx2, sy2), 'depth': avg_d, 'color': '#e0e0e0', 'width': 2, 'dash': (4,4)})
            render_list.append({'type': 'text', 'coords': (sx2, sy2), 'depth': d2, 'text': label, 'color': 'gray'})

        if joints:
            sx, sy, d = self.project_3d(*joints[0], az, el)
            render_list.append({'type': 'dot', 'coords': (sx, sy), 'depth': d, 'color': 'black' if not color_dim else 'gray', 'r': self.get_perspective_radius(d)})
            
            colors = ['#FF4444', '#44AA44', '#4444FF']
            
            for i in range(3):
                sx1, sy1, d1 = self.project_3d(*joints[i], az, el)
                sx2, sy2, d2 = self.project_3d(*joints[i+1], az, el)
                
                line_color = colors[i] if not color_dim else '#cccccc'
                avg_d = (d1 + d2) / 2
                
                render_list.append({'type': 'line', 'coords': (sx1, sy1, sx2, sy2), 'depth': avg_d, 'color': line_color, 'width': 4, 'dash': None})
                render_list.append({'type': 'dot', 'coords': (sx2, sy2), 'depth': d2, 'color': line_color, 'r': self.get_perspective_radius(d2)})

        tgt_sx, tgt_sy, tgt_d = self.project_3d(tx, ty, tz, az, el)
        self.canvas.create_line(tgt_sx-6, tgt_sy-6, tgt_sx+6, tgt_sy+6, fill="magenta", width=2)
        self.canvas.create_line(tgt_sx-6, tgt_sy+6, tgt_sx+6, tgt_sy-6, fill="magenta", width=2)

        render_list.sort(key=lambda item: item['depth'], reverse=True)
        
        for item in render_list:
            if item['type'] == 'line':
                x1, y1, x2, y2 = item['coords']
                self.canvas.create_line(x1, y1, x2, y2, fill=item['color'], width=item['width'], dash=item['dash'])
            elif item['type'] == 'dot':
                sx, sy = item['coords']
                r = item['r']
                self.canvas.create_oval(sx - r, sy - r, sx + r, sy + r, fill=item['color'], outline="black", width=1)
            elif item['type'] == 'text':
                sx, sy = item['coords']
                self.canvas.create_text(sx, sy, text=item['text'], fill=item['color'], font=("Arial", 10, "bold"))

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotLegApp(root)
    root.mainloop()