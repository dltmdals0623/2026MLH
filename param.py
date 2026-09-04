import numpy as np
import pickle

HSV_PATH="hsv_sav.pkl"

MIN_DETECT_AREA =   300

MIN_LINE_HSV =   np.array([  25, 130, 164])
MAX_LINE_HSV =   np.array([  33, 255, 219])

MIN_OBJ_HSV =   np.array([ 74, 100,  69])
MAX_OBJ_HSV =   np.array([ 84, 255, 145])

try:
    with open(HSV_PATH, "rb") as f:
        hsv_dict = pickle.load(f)
    print("\033[32mHSV preset Found\033[0m\n")
    print(' '*14+f"\033[32m{'Current HSV':^13}    \033[31m{'Incoming HSV':^13}\033[0m\nMIN LINE HSV: \033[32m{MIN_LINE_HSV}\033[0m -> \033[31m{hsv_dict['min_line_hsv']}\033[0m\nMAX LINE HSV: \033[32m{MAX_LINE_HSV}\033[0m -> \033[31m{hsv_dict['max_line_hsv']}\033[0m\nMIN OBJ HSV:  \033[32m{MIN_OBJ_HSV}\033[0m -> \033[31m{hsv_dict['min_obj_hsv']}\033[0m\nMAX OBJ HSV:  \033[32m{MAX_OBJ_HSV}\033[0m -> \033[31m{hsv_dict['max_obj_hsv']}\033[0m")
    res=input("Which HSV do you want to accept? (\033[32mCurrent(c)\033[0m or \033[31mIncoming(i)\033[0m)\n-> ")
    while 1:
        if res.lower() in ['current','c']:
            MIN_LINE_HSV = hsv_dict['min_line_hsv']
            MAX_LINE_HSV = hsv_dict['max_line_hsv']

            MIN_OBJ_HSV  = hsv_dict['min_obj_hsv']
            MAX_OBJ_HSV  = hsv_dict['max_obj_hsv']
            print('\033[32mCurrent HSV Accepted\033[0m')
            break
        elif res.lower() in ['incoming','i']:
            MIN_LINE_HSV = hsv_dict['min_line_hsv']
            MAX_LINE_HSV = hsv_dict['max_line_hsv']

            MIN_OBJ_HSV  = hsv_dict['min_obj_hsv']
            MAX_OBJ_HSV  = hsv_dict['max_obj_hsv']
            print('\033[32mIncoming HSV Accepted\033[0m')
            break
        else:
            res=input('Please write ONLY \033[32m"CURRENT"\033[0m or \033[31m"INCOMING"\033[0m.\n-> ')

except:
    print("\033[31mHSV preset Unfound\033[0m")

UPPER_LINE_MASK_LIMIT = 0.70
LOWER_LINE_MASK_LIMIT = 0.85
LEFT_LINE_MASK_LIMIT  = 0.05
RIGHT_LINE_MASK_LIMIT = 0.95

CURRENT_LINE = 1.20
GROUP_X_MARGIN= 5
SOBEL_THRESHOLD = 80

L1=27
L2=37
L3=82
