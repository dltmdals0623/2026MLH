import tkinter as tk
import math
import packet as pk
import param as p



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
        y1_j2 * math.sin(th1) + z1_j2 * math.cos(th1))

    x1_j3 = p.L2 * math.sin(th2) + p.L3 * math.sin(th2 - th3)
    z1_j3 = -(p.L2 * math.cos(th2) + p.L3 * math.cos(th2 - th3))
    y1_j3 = p.L1
    p3 = (
        x1_j3,
        y1_j3 * math.cos(th1) - z1_j3 * math.sin(th1),
        y1_j3 * math.sin(th1) + z1_j3 * math.cos(th1))

    return [p0, p1, p2, p3]





def cal_leg_offset(delta_xyz, inv=0):
    if delta_xyz == [0,0,0]:
        return [512,512,512]
    ik_res = calculate_ik_3dof(9+delta_xyz[0],27+delta_xyz[1],-108+delta_xyz[2])
    if ik_res is None:
        return None
    offset1 = (lambda th: (512 - int((math.degrees(th) + 180.0) * (1024 / 360)))  +512)(ik_res[0])
    offset2 = (lambda th: (653 - int((math.degrees(th) + 180.0) * (1024 / 360))) *(1 -2*inv) +512)(ik_res[1])
    offset3 = (lambda th: (696 - int((math.degrees(th) + 180.0) * (1024 / 360))) *(1 -2*inv) +512)(ik_res[2])

    return [offset1, offset2, offset3]


def cal_offset(id,delta_x):
    if not delta_x:
        return [(i,512) for i in id]
    fl=cal_leg_offset([delta_x,0,0],0)
    fr=cal_leg_offset([-delta_x,0,0],1)
    bl=cal_leg_offset([-delta_x,0,0],0)
    br=cal_leg_offset([delta_x,0,0],1)

    offsets=[]
    for i in range(3):
        if 1+4*i in id: offsets.append((1+4*i, fl[i]))
        if 2+4*i in id: offsets.append((2+4*i, fr[i]))
        if 3+4*i in id: offsets.append((3+4*i, bl[i]))
        if 4+4*i in id: offsets.append((4+4*i, br[i]))

    return offsets