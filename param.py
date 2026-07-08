import numpy as np


MIN_DETECT_AREA =   3000                   #반영할 컨투어 최소 넓이

# MIN_LINE_HSV =   np.array([15,  50, 57])    #라인 최소 HSV
# MAX_LINE_HSV =   np.array([35, 217, 183])  #라인 최대 HSV

MIN_LINE_HSV =   np.array([20,  53, 160])    #라인 최소 HSV
MAX_LINE_HSV =   np.array([31, 231, 255])  #라인 최대 HSV

UPPER_LINE_MASK_LIMIT = 0.30               #라인 감지 한계_상
LOWER_LINE_MASK_LIMIT = 1.00               #라인 감지 한계_하

CURRENT_LINE = 1.50               #로봇 추정 위치 (세타값 중심점)
