import time
import serial
import logging

logging.basicConfig(level=logging.INFO, format="[%(levelname)s %(lineno)d] %(message)s")
PORT="COM3"

try:
    ser = serial.Serial(port=PORT,
                        baudrate=57600,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,   
                        bytesize=serial.EIGHTBITS,
                        timeout=0)
    ser_enabled = True
except serial.SerialException as e:
    # logging.warning(f"SerialError: {str(e)}")
    ser_enabled = False


#메시지 강조 함수
def stress(msg,nl,color='\033[0m'):
    if nl == 0:
        print()
    print(color+"="*60)
    print("{:^60}".format(msg))
    print("="*60+'\033[0m')
    if nl == 2:
        print()


#단순 시리얼 전송 함수
def serial_raw(val):
    down_byte = val & 0xFF
    up_byte = (val >> 8) & 0xFF
    try:
        ser.write(bytearray([255, 85, down_byte, 255-down_byte, up_byte, 255-up_byte]))
    except serial.SerialException as e:
        logging.error(f"SerialError: {str(e)}")


#키, 데이터로 시리얼 전송하는 함수
def serial_write(key, data):
    if key < 0 or key > 31 or data < 0 or data > 2046:
        if key < 0 or key > 31:
            logging.error(f"key '{key}' is out of range (0~31)")
        if data < 0 or data > 2046:
            logging.error(f"data '{data}' is out of range (0~2046)")
        return False
    # print(key, data)
    packed_val = (key << 11) | data
    serial_raw(packed_val)
    return packed_val


#입력받은 응답값 기다리는 함수
def wait_ack(expected_ack=65535, timeout=3.0):
    start_time = time.time()
    while time.time() - start_time < timeout:
        if ser.in_waiting >= 6:
            packet = ser.read(6)
            if packet[0] == 0xFF and packet[1] == 0x55:
                received_val = (packet[4] << 8) | packet[2]
                if received_val == expected_ack:
                    return True
    return False


#튜플 가변인자로 패킷 수신 함수
def packet_write(*args):
    if not ser_enabled:
        return False
    serial_raw(65535)                                   #패킷 시작 신호를 보낸다
    if not wait_ack():
        logging.error("Packet [START] ACK Timeout!")
        return False
    
    for key,data in args:
        val = serial_write(key,data)
        if not wait_ack(val):
            logging.error(f"Packet [{key}:{data}] ACK Timeout!")
            return False

    serial_raw(65535)                                   #패킷 끝 신호를 보낸다
    if not wait_ack():
        logging.error("Packet [END] ACK Timeout!")
        return False
    
    return True


def run():
    if not ser_enabled:                         #포트가 안 열려 있으면 종료한다
        stress(f"COULD NOT OPENS {PORT}!",1)
        return

    stress("TRANSMIT START!",2, '\033[32m')

    try:
        total_start_time = time.time()
        for i in range(100):                                #패킷 100개를 보낸다
            start_time=time.time()
            if not packet_write(*[(n%31, i) for n in range(100, 200)]):
                return

            end_time=time.time()
            t = end_time - start_time
            print('\r>'+"{:^59}".format(f"Packet [{i+1}/100] SUCCEED! | TIME SPENT: {t:.3f}sec"),end='')
        T=end_time-total_start_time
        print()

        stress(f"TOTAL : {time.time() - total_start_time:.3f}sec | PPS : {100/T:.3f}packet | DPS : {10000/T:.3f}data",0,'\033[32m')

    except KeyboardInterrupt:
        print()
        stress("Terminated by USER",0,'\033[31m')
    finally:
        if ser.is_open:
            ser.close()


if __name__ == "__main__":
    run()