from csicam import CsiCamCapture
import numpy as np
import param as p
import pickle
import cv2
import os

if os.name == 'nt':
    cap = cv2.VideoCapture(0)
else:
    cap = CsiCamCapture(0)

try:
    with open(p.HSV_PATH, "rb") as f:
        hsv_dict = pickle.load(f)
    print("파일 찾음")
except:
    print("파일 못찾음")
    with open(p.HSV_PATH, "wb") as f:
        hsv_dict={
            "min_line_hsv":np.array([25, 130, 164]),
            "max_line_hsv":np.array([33, 255, 219]),
            "min_obj_hsv":np.array([74, 100, 69]),
            "max_obj_hsv":np.array([84, 255, 145])
        }
        pickle.dump(hsv_dict, f)

print(hsv_dict)
cam_w = 0
cam_h = 0

hsv_val=np.array([0,0,0,0,0,0])
l=np.array([])
cnt=0

cv2.namedWindow("blended")


def mouse_event(event, x, y, _, param):
    if event == cv2.EVENT_FLAG_LBUTTON:
        global hsv_val, cnt

        if cnt == 0:
            hsv_val=np.append(hsv[y,x],hsv[y,x])
            print(hsv_val)

        tmp1=hsv_val[:3]
        tmp2=hsv_val[3:]
        if cnt !=0:

            tmp1=np.minimum(tmp1, hsv[y,x])
            tmp2=np.maximum(tmp2, hsv[y,x])
            hsv_val=np.append(tmp1,tmp2)
            print(hsv[y,x], hsv_val)

        print(f"MIN HSV CHANGED 2 {hsv_val[:3]}")
        print(f"MAX HSV CHANGED 2 {hsv_val[3:]}")
        cnt+=1

    return

while True:
    cam_ret, dst = cap.read()
    # print('\r',type(hsv_val[:3]), type(hsv_val[3:]),hsv_val[:3],hsv_val[3:],end='')
    if not cam_ret:
        print('\r'+"\033[31mCameraError: Failed to capture camera\033[0m",end='')
        continue
    dst = cv2.GaussianBlur(dst, (0, 0), 5)
    hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
    cam_h, cam_w = dst.shape[:2]

    mask = cv2.inRange(hsv, hsv_val[:3], hsv_val[3:])
    mask2 = np.zeros_like(dst)
    mask2[mask > 0] = [127, 255, 255]
    cv2.setMouseCallback("blended", mouse_event, dst)

    key= cv2.waitKey(1)
    if key == ord('r'):
        cnt=0
        hsv_val=np.array([0,0,0,0,0,0])

    elif key == ord('q'):
        break

    elif key == ord('l'):
        res=input("Will you save this val in \033[33mLINE_HSV\033[0m? (\033[32mY\033[0m or \033[31mN\033[0m)\n-> ")
        while 1:
            if res.lower() == 'y':
                hsv_dict["min_line_hsv"] = np.array([*hsv_val[:3]])
                hsv_dict["max_line_hsv"] = np.array([*hsv_val[3:]])
                with open(p.HSV_PATH, "wb") as f:
                    pickle.dump(hsv_dict, f)
                print('LINE_HSV saved successfully')
                break
            elif res.lower() == 'n':
                print('LINE_HSV saving Canceled')
                break
            else:
                res=input('Please write ONLY \033[32m"Y"\033[0m or \033[31m"N"\033[0m.\n-> ')

    elif key == ord('o'):
        res=input("Will you save this val in \033[34mOBJ_HSV\033[0m? (\033[32mY\033[0m or \033[31mN\033[0m)\n-> ")
        while 1:
            if res.lower() == 'y':
                hsv_dict["min_obj_hsv"] = np.array([*hsv_val[:3]])
                hsv_dict["max_obj_hsv"] = np.array([*hsv_val[3:]])
                with open(p.HSV_PATH, "wb") as f:
                    pickle.dump(hsv_dict, f)
                print('LINE_HSV saved successfully')
                break
            elif res.lower() == 'n':
                print('LINE_HSV saving Canceled')
                break
            else:
                res=input('Please write ONLY \033[32m"Y"\033[0m or \033[31m"N"\033[0m.\n-> ')




    if mask.shape is not None:
        blended = cv2.addWeighted(dst, 1.0, mask2, 1, 0)

    cv2.imshow("blended", blended)
cv2.destroyAllWindows()
