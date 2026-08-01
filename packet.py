import time
import serial
import param as p

ser_enabled = False

try:
    ser = serial.Serial(port="COM3",
                        baudrate=57600,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,   
                        bytesize=serial.EIGHTBITS,
                        timeout=0)
    ser_enabled = True
except serial.SerialException as e:
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
        print(f"SerialError: {str(e)}")



#키, 데이터로 시리얼 전송하는 함수
def serial_write(key, data):
    if key < 0 or key > 31 or data < 0 or data > 2046:
        if key < 0 or key > 31:
            print(f"key '{key}' is out of range (0~31)")
        if data < 0 or data > 2046:
            print(f"data '{data}' is out of range (0~2046)")
        return False
    packed_val = (key << 11) | data
    serial_raw(packed_val)
    return packed_val



def wait_ack(expected_ack=65535, timeout=0.5):
    if not ser_enabled:
        return False

    start_time = time.time()
    state = 0
    expected_low = expected_ack & 0xFF
    expected_high = (expected_ack >> 8) & 0xFF

    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            b = ser.read(1)
            if not b:
                continue
            byte = b[0]
            if state == 0:
                if byte == 0xFF: 
                    state = 1
            elif state == 1:
                if byte == 0x55: 
                    state = 2
            elif state == 2:   # Data_L (실제 데이터 하위 바이트 저장)
                if byte == expected_low:
                    state = 3
            elif state == 3:   # Data_H (실제 데이터 상위 바이트 저장)
                if byte == expected_high:
                    return True
        else:
            time.sleep(0.001)

    return False

#튜플 가변인자로 패킷 수신 함수
def packet_write(*args):
    if ser_enabled == False:
        return [0,0,0]
    serial_raw(65535)
    if not wait_ack():
        return [-2,0,0]

    for key,data in args:
        val = serial_write(key,data)
        if not wait_ack(val):
            return [-1,key,data]

    serial_raw(65535)
    if not wait_ack():
        return [-3,0,0]
    
    return [1,0,0]


def run():
    if not ser_enabled:
        stress(f"COULD NOT OPENS {ser.port}!",1)
        return

    stress("TRANSMIT START!",2, '\033[32m')

    try:
        total_start_time = time.time()
        for i in range(100):
            pk_ret, pk_key, pk_data = packet_write(*[(n%31, i) for n in range(0, 100)])
            if pk_ret != 1:
                match(pk_ret):
                    case 0:
                        print("SERIAL NOT AVAILABLE",end='')
                    # case -1:
                        print(f"Packet [{pk_key}:{pk_data}] 그저 Goa", end=' ')
                    case -2:
                        print("Packet [START] ACK Timeout!",end='')
                    case -3:
                        print(U"Packet [END] ACK Timeout!",end='')

            print('\r>'+"{:^59}".format(f"Packet [{i+1}/100] SUCCEED!"),end='')
        T=time.time()-total_start_time
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