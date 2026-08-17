import cv2
from CsiCamCapture import CsiCamCapture



cap = CsiCamCapture(0)

while True:
    ret, src = cap.read()
    if not ret: continue

    cv2.imshow("CSI Camera Test", src)

    if cv2.waitKey(1) == ord("q"): break



cap.release()
cv2.destroyAllWindows()