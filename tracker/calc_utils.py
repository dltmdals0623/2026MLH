def rect_center(rect):      #사각형 중앙 좌표 계산 함수
    x, y, w, h = rect           #사각형 튜플에서 좌표, 크기 추출
    return (x + w/2, y + h/2)   #사각형 중앙 좌표 리턴


def distance(pos1, pos2):                               #좌표 거리 계산 함수
    x_valid = pos1[0] is not None and pos2[0] is not None   #pos1과 pos2의 x좌표 존재 여부
    y_valid = pos1[1] is not None and pos2[1] is not None   #pos1과 pos2의 y좌표 존재 여부

    if x_valid and y_valid:                                         #x, y좌표가 모두 존재하면
        return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2) ** 0.5 #피타고라스 정리로 계산한 두 점의 거리 리턴
    if x_valid:                                                     #x좌표만 존재하면
        return abs(pos1[0] - pos2[0])                                   #두 점의 x좌표 거리 리턴
    if y_valid:                                                     #y좌표만 존재하면
        return abs(pos1[1] - pos2[1])                                   #두 점의 y좌표 거리 리턴
    print("\033[32mSortError: Cannot calculate distance\033[0m")           #x, y좌표가 모두 없으면 에러레벨로 로그 출력
    return None                                                     #빈 값 리턴        