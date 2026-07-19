import tkinter as tk
import math
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ==========================================
# 1. 역기구학(IK) 및 정기구학(FK) 수식 (변경 없음)
# ==========================================
def calculate_ik_3dof(x, y, z, L1, L2, L3):
    try:
        R_yz = math.sqrt(y**2 + z**2)
        if R_yz < L1:
            raise ValueError("기저 반경보다 가까움")

        z1 = -math.sqrt(R_yz**2 - L1**2)
        phi = math.atan2(z1, L1)
        psi = math.atan2(z, y)
        theta_1 = psi - phi

        x1 = x
        original_D = math.sqrt(x1**2 + z1**2)
        
        max_reach = L2 + L3
        min_reach = abs(L2 - L3)

        if original_D - max_reach > 1:
            return None
        elif original_D > max_reach:
            D = max_reach
        elif original_D < min_reach:
            D = min_reach
        else:
            D = original_D

        cos_theta_3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)
        cos_theta_3 = max(-1.0, min(1.0, cos_theta_3))
        theta_3 = math.acos(cos_theta_3)

        alpha = math.atan2(x1, -z1)
        cos_beta = (L2**2 + D**2 - L3**2) / (2 * L2 * D)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)

        theta_2 = alpha - beta
        return theta_1, theta_2, theta_3

    except ValueError:
        return None

def get_joint_positions(th1, th2, th3, L1, L2, L3):
    p0 = (0, 0, 0)
    p1 = (0, L1 * math.cos(th1), L1 * math.sin(th1))

    x1_j2 = L2 * math.sin(th2)
    z1_j2 = -L2 * math.cos(th2)
    y1_j2 = L1
    p2 = (
        x1_j2,
        y1_j2 * math.cos(th1) - z1_j2 * math.sin(th1),
        y1_j2 * math.sin(th1) + z1_j2 * math.cos(th1)
    )

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
# 2. 모터 오프셋 계산 함수 (변경 없음)
# ==========================================
def cal_joint_offset(current_joints, delta_xyz, L1=46.5, L2=38.0, L3=42.0):
    rad_joints = []
    for step in current_joints:
        deg = (step * (360.0 / 1024.0)) - 180.0
        rad = math.radians(deg)
        rad_joints.append(rad)
        
    current_th1, current_th2, current_th3 = rad_joints
    dx, dy, dz = delta_xyz

    positions = get_joint_positions(current_th1, current_th2, current_th3, L1, L2, L3)
    current_x, current_y, current_z = positions[3] 

    target_x = current_x + dx
    target_y = current_y + dy
    target_z = current_z + dz

    ik_result = calculate_ik_3dof(target_x, target_y, target_z, L1, L2, L3)
    if ik_result is None:
        return None

    new_th1, new_th2, new_th3 = ik_result
    new_steps = []
    for rad in (new_th1, new_th2, new_th3):
        deg = math.degrees(rad) + 180.0
        step = round(deg * (1024/360))
        step = max(0, min(1023, step))
        new_steps.append(step)

    d_th1 = new_steps[0] - current_joints[0]+512
    d_th2 = new_steps[1] - current_joints[1]+512
    d_th3 = new_steps[2] - current_joints[2]+512
    return (d_th1, d_th2, d_th3)

# ==========================================
# 3. Tkinter + Matplotlib 3D GUI 시각화 클래스
# ==========================================
class RobotLegApp:
    def __init__(self, root):
        self.root = root
        self.root.title("3DOF 로봇 다리 IK 3D 시각화 (Matplotlib 버전)")
        
        self.L1, self.L2, self.L3 = 46.5, 38.0, 42.0
        self.prev_joints = None
        
        self.init_ui()
        self.update_view()

    def init_ui(self):
        # 왼쪽 컨트롤 패널 (기존과 동일)
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
        
        tk.Label(ctrl_frame, text="------------------------------").pack(pady=5)
        tk.Label(ctrl_frame, text="관절 출력 각도 (기준 적용)", font=("Arial", 11, "bold")).pack(pady=5)
        
        self.angle1_var = tk.StringVar()
        self.angle2_var = tk.StringVar()
        self.angle3_var = tk.StringVar()
        
        tk.Label(ctrl_frame, textvariable=self.angle1_var, fg="#FF4444", font=("Arial", 11, "bold")).pack(pady=2)
        tk.Label(ctrl_frame, textvariable=self.angle2_var, fg="#44AA44", font=("Arial", 11, "bold")).pack(pady=2)
        tk.Label(ctrl_frame, textvariable=self.angle3_var, fg="#4444FF", font=("Arial", 11, "bold")).pack(pady=2)

        self.status_var = tk.StringVar()
        self.status_label = tk.Label(ctrl_frame, textvariable=self.status_var, font=("Arial", 11, "bold"))
        self.status_label.pack(pady=20)
        
        # 오른쪽 캔버스 (Tkinter Canvas 대신 Matplotlib FigureCanvasTkAgg 사용)
        self.fig = Figure(figsize=(7, 7), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # 그래프 여백 최소화
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

    def update_view(self, *_):
        # 목표 좌표 및 뷰 앵글 가져오기
        tx, ty, tz = self.scale_x.get(), self.scale_y.get(), self.scale_z.get()
        az, el = self.scale_az.get(), self.scale_el.get()
        
        ik_res = calculate_ik_3dof(tx, ty, tz, self.L1, self.L2, self.L3)
        
        # 축 초기화
        self.ax.cla()
        
        if ik_res is None:
            self.status_var.set("상태: 도달 불가 (Workspace 이탈)")
            self.status_label.config(fg="red")
            self.angle1_var.set("Joint 1: Error")
            self.angle2_var.set("Joint 2: Error")
            self.angle3_var.set("Joint 3: Error")
            joints = self.prev_joints
            color_dim = True
        else:
            self.status_var.set("상태: 정상 도달")
            self.status_label.config(fg="black")
            th1, th2, th3 = ik_res
            
            disp_th1 = math.degrees(th1) + 180.0
            disp_th2 = math.degrees(th2) + 180.0
            disp_th3 = math.degrees(th3) + 180.0
            
            self.angle1_var.set(f"Joint 1 (Red): {disp_th1:.1f}°, {disp_th1*(1024/360):.0f}")
            self.angle2_var.set(f"Joint 2 (Green): {disp_th2:.1f}° {disp_th2*(1024/360):.0f}")
            self.angle3_var.set(f"Joint 3 (Blue): {disp_th3:.1f}° {disp_th3*(1024/360):.0f}")
            
            joints = get_joint_positions(*ik_res, self.L1, self.L2, self.L3)
            self.prev_joints = joints
            color_dim = False

        # 3D 공간 축 범위 고정 (그래프가 널뛰는 현상 방지)
        axis_limit = 120
        self.ax.set_xlim([-axis_limit, axis_limit])
        self.ax.set_ylim([-axis_limit, axis_limit])
        self.ax.set_zlim([-axis_limit, axis_limit])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        # 목표 지점 (마젠타색 'x' 표시)
        self.ax.scatter(tx, ty, tz, color='magenta', marker='x', s=100, label='Target')

        # 로봇 다리 그리기
        if joints:
            xs = [p[0] for p in joints]
            ys = [p[1] for p in joints]
            zs = [p[2] for p in joints]
            
            # 상태에 따른 색상 설정
            colors = ['#FF4444', '#44AA44', '#4444FF'] if not color_dim else ['gray', 'gray', 'gray']
            
            # Base 점 찍기
            self.ax.scatter(xs[0], ys[0], zs[0], color='black', s=50)

            # 링크 및 관절 점 그리기
            for i in range(3):
                # 선 (링크)
                self.ax.plot(xs[i:i+2], ys[i:i+2], zs[i:i+2], color=colors[i], linewidth=5)
                # 점 (조인트)
                self.ax.scatter(xs[i+1], ys[i+1], zs[i+1], color=colors[i], s=50)

        # 뷰 앵글 적용 (슬라이더 값 연동)
        # self.ax.view_init(elev=el, azim=az)
        
        # 캔버스 다시 그리기
        self.canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotLegApp(root)
    root.mainloop()