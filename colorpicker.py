from csicam import CsiCamCapture
import numpy as np
import cv2
import os

if os.name == 'nt':
    cap = cv2.VideoCapture(0)
else:
    cap = CsiCamCapture(0)

cam_w = 0
cam_h = 0

hsv_val=np.array([0,0,0,0,0,0])
l=np.array([])
cnt=0

cv2.namedWindow("blended")


def mouse_event(event, x, y, _, param):
    if event == cv2.EVENT_FLAG_LBUTTON: #마우스 좌클릭 시 커서가 가리키는 좌표 hsv값과 비교해서 최대 최소 hsv 업데이트
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

    mask = cv2.inRange(hsv, hsv_val[:3], hsv_val[3:])   #마스크 영역 원본 이미지에 오버레이
    mask2 = np.zeros_like(dst)
    mask2[mask > 0] = [127, 255, 255]
    cv2.setMouseCallback("blended", mouse_event, dst)

    key= cv2.waitKey(1)
    if key == ord('r'):
        cnt=0
        hsv_val=np.array([0,0,0,0,0,0])
    elif key == ord('q'):
        break

    if mask.shape is not None:
        blended = cv2.addWeighted(dst, 1.0, mask2, 0.5, 0)

    # cv2.imshow("res", mask)
    # cv2.imshow("dst", dst)
    cv2.imshow("blended", blended)
cv2.destroyAllWindows()
