import numpy as np


MIN_DETECT_AREA =   100                   #반영할 컨투어 최소 넓이

# MIN_LINE_HSV =   np.array([0,   0, 0])    #라인 최소 HSV
# MAX_LINE_HSV =   np.array([255, 150, 150])  #라인 최대 HSV

MIN_LINE_HSV =   np.array([20,  30, 100])    #라인 최소 HSV
MAX_LINE_HSV =   np.array([40, 255, 200])  #라인 최대 HSV

UPPER_LINE_MASK_LIMIT = 0.10              #라인 감지 한계_상
LOWER_LINE_MASK_LIMIT = 1.00              #라인 감지 한계_하
LEFT_LINE_MASK_LIMIT  = 0.05              #물체 감지 한계_좌
RIGHT_LINE_MASK_LIMIT = 0.95              #물체 감지 한계_우

CURRENT_LINE = 1.50               #로봇 추정 위치 (세타값 중심점)

SOBEL_THRESHOLD = 200         # Sobel 엣지 이진화 임계값
GROUP_X_MARGIN = 50          # 같은 라인으로 묶을 최대 x거리

L1=27
L2=38.0 
L3=86.0