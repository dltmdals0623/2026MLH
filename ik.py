import tkinter as tk
import math
import packet as pk
import param as p

# ==========================================
# 1. 역기구학(IK) 및 정기구학(FK) 수식 
# ==========================================
def calculate_ik_3dof(x, y, z):
    try:
        R_yz = math.sqrt(y**2 + z**2)
        if R_yz < p.L1:
            raise ValueError("기저 반경보다 가까움")

        z1 = -math.sqrt(R_yz**2 - p.L1**2)
        phi = math.atan2(z1, p.L1)
        psi = math.atan2(z, y)
        theta_1 = psi - phi

        x1 = x

        original_D = math.sqrt(x1**2 + z1**2)
        
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
        theta_3 = math.acos(cos_theta_3)

        alpha = math.atan2(x1, -z1)
        cos_beta = (p.L2**2 + D**2 - p.L3**2) / (2 * p.L2 * D)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)

        theta_2 = alpha + beta 

        return theta_1, theta_2, theta_3

    except ValueError:
        return None


def get_joint_positions(th1, th2, th3):
    p0 = (0, 0, 0)

    p1 = (0, p.L1 * math.cos(th1), p.L1 * math.sin(th1))

    x1_j2 = p.L2 * math.sin(th2)
    z1_j2 = -p.L2 * math.cos(th2)
    y1_j2 = p.L1
    p2 = (
        x1_j2,
        y1_j2 * math.cos(th1) - z1_j2 * math.sin(th1),
        y1_j2 * math.sin(th1) + z1_j2 * math.cos(th1)
    )

    x1_j3 = p.L2 * math.sin(th2) + p.L3 * math.sin(th2 - th3)
    z1_j3 = -(p.L2 * math.cos(th2) + p.L3 * math.cos(th2 - th3))
    y1_j3 = p.L1
    p3 = (
        x1_j3,
        y1_j3 * math.cos(th1) - z1_j3 * math.sin(th1),
        y1_j3 * math.sin(th1) + z1_j3 * math.cos(th1)
    )

    return [p0, p1, p2, p3]



# ==========================================
# 2. 모터 오프셋 계산 함수
# ==========================================
def cal_leg_offset(delta_xyz, inv=0):
    # if delta_xyz == [0,0,0]:
    #     return [512,512,512]
    ik_res = calculate_ik_3dof(9+delta_xyz[0],27+delta_xyz[1],-108+delta_xyz[2])
    if ik_res is None:
        return None
    offset1 = (lambda th: (512 - int((math.degrees(th) + 180.0) * (1024 / 360)))  +512)(ik_res[0])
    offset2 = (lambda th: (653 - int((math.degrees(th) + 180.0) * (1024 / 360))) *(1 -2*inv) +512)(ik_res[1])
    offset3 = (lambda th: (696 - int((math.degrees(th) + 180.0) * (1024 / 360))) *(1 -2*inv) +512)(ik_res[2])

    return [offset1, offset2, offset3]

def cal_offset(delta_x):
    if not delta_x:
        return [(i,512) for i in range(1, 13)]
    fl=cal_leg_offset([delta_x,0,0],0)
    fr=cal_leg_offset([-delta_x,0,0],1)
    bl=cal_leg_offset([-delta_x,0,0],0)
    br=cal_leg_offset([delta_x,0,0],1)

    # print(fl)
    # print(fr)
    # print(bl)
    # print(br)
    offsets=[]
    for i in range(3):
        offsets.append((1+4*i, fl[i]))
        offsets.append((2+4*i, fr[i]))
        offsets.append((3+4*i, bl[i]))
        offsets.append((4+4*i, br[i]))

    return offsets

# ==========================================
# 3. Tkinter 3D GUI 시각화 클래스
# ==========================================
class RobotLegApp:
    def __init__(self, root):
        self.root = root
        self.root.title("3DOF 로봇 다리 IK 3D 시각화")
        self.prev_joints = None

        self.init_ui()
        self.update_view()


    def init_ui(self):
        # 왼쪽 컨트롤 패널 (슬라이더 및 출력)
        ctrl_frame = tk.Frame(self.root, padx=15, pady=15)
        ctrl_frame.pack(side=tk.LEFT, fill=tk.Y)

        tk.Label(ctrl_frame, text="목표 좌표 (Target XYZ)", font=("Arial", 11, "bold")).pack(pady=5)
        self.scale_x = tk.Scale(ctrl_frame, from_=-100, to=100, orient=tk.HORIZONTAL, label="X 좌표", length=220, command=self.update_view)
        self.scale_x.set(9)
        self.scale_x.pack()

        self.scale_y = tk.Scale(ctrl_frame, from_=-20, to=150, orient=tk.HORIZONTAL, label="Y 좌표", length=220, command=self.update_view)
        self.scale_y.set(27)
        self.scale_y.pack()

        self.scale_z = tk.Scale(ctrl_frame, from_=-120, to=-50, orient=tk.HORIZONTAL, label="Z 좌표", length=220, command=self.update_view)
        self.scale_z.set(-108)
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

        self.canvas_w, self.canvas_h = 700, 800
        self.canvas = tk.Canvas(self.root, width=self.canvas_w, height=self.canvas_h, bg="#ffffff", relief="sunken", bd=2)
        self.canvas.pack(side=tk.RIGHT, padx=10, pady=10)


    def project_3d(self, x, y, z, az, el, scale=3.5):
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

        ik_res = calculate_ik_3dof(tx, ty, tz)

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

            joints = get_joint_positions(*ik_res)
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
            # print(*[(key, data) for key, data in zip([1,5,9,2,6,10,3,7,11,4,8,12],[*cal_joint_offset(joints[3],0)+cal_joint_offset(joints[3],1)]*2)])
            # pk.packet_write(*[(key, data) for key, data in zip([1,5,9,2,6,10,3,7,11,4,8,12],[*cal_joint_offset(joints[3],0)+cal_joint_offset(joints[3],1)]*2)])
            cal_offset(self.scale_x.get())

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