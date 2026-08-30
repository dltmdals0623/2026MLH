import numpy as np

MIN_DETECT_AREA =   1000

MIN_LINE_HSV =   np.array([  25, 130, 164])    #라인 최소 HSV
MAX_LINE_HSV =   np.array([  33, 255, 219])  #라인 최대 HSV

MIN_OBJ_HSV =   np.array([ 74, 100,  69])    #라인 최소 HSV
MAX_OBJ_HSV =   np.array([ 84, 255, 145])  #라인 최대 HSV

UPPER_LINE_MASK_LIMIT = 0.70              #라인 감지 한계_상
LOWER_LINE_MASK_LIMIT = 0.85              #라인 감지 한계_하
LEFT_LINE_MASK_LIMIT  = 0.05              #물체 감지 한계_좌
RIGHT_LINE_MASK_LIMIT = 0.95              #물체 감지 한계_우

CURRENT_LINE = 1.20               #로봇 추정 위치 (세타값 중심점)
GROUP_X_MARGIN= 5
SOBEL_THRESHOLD = 80

L1=27
L2=37
L3=82
