MIN_DETECT_AREA =   50

# MIN_LINE_HSV =   np.array([0,   0, 0])    #라인 최소 HSV
# MAX_LINE_HSV =   np.array([255, 150, 150])  #라인 최대 HSV

UPPER_LINE_MASK_LIMIT = 0.50
LOWER_LINE_MASK_LIMIT = 0.80
LEFT_LINE_MASK_LIMIT  = 0.10
RIGHT_LINE_MASK_LIMIT = 0.90

CURRENT_LINE = 1.20
GROUP_X_MARGIN= 50
SOBEL_THRESHOLD = 80

L1=27
L2=37
L3=82



if __name__ == "__main__":
    import datetime
    import time

    def stress(msg,nl,color='\033[0m'):
        if nl == 0:
            print()
        print(color+"="*50)
        print("{:^50}".format(msg))
        print("="*50+'\033[0m')
        if nl == 2:
            print()

    def tktime():
        now=str(datetime.datetime.now()).split()

        now[0]=now[0].split('-')
        now[1]=now[1].split(':')
        sec=now[1][2].split('.')
        now[1][2] = sec[0]
        now[1].append(sec[1])
        return now

    try:
        while True:
            now = tktime()
            print('\r'+f"지금은 {now[0][0]}년 {now[0][1]}월 {now[0][2]}일 {now[1][0]}시 {now[1][1]}분 {now[1][2]}초 {now[1][3][:2]} 입니다.",end='')
            time.sleep(0.005)

        # score=[70,75,85,90,95]
        # score.append(80)
        # score.sort()
        # print(score)

        # score.remove(90)
        # print(score)

        # mov=['오디세이','스파이더맨: 브랜드 뉴 데이', '경주기행', '오크 스트리트의 마지막 날', '인시디어스: 그들이 넘어왔다']
        # mov.append('사랑의 하츄핑: 고래보석의 전설')
        # mov.insert(2,'호프')
        # del mov[1]
        # print(len(mov))
        # print(mov)

    except:
#     print("BOX ON\n")
#     time.sleep(1)
#     st=time.time()
#     nt=time.time()
#     pre=None
#     while nt-st<=10:
#         nt=time.time()
#         out=int(11-(nt-st))
#         if out != pre:
#             print(out)
#             pre=out
#         time.sleep(0.001)

    # score=0
    # for i in ['국어','영어','수학']: score+=float(input(f"{i} 성적: "))
    # print(score/3)


        stress("Interrupted by USSR",0,'\033[31m')