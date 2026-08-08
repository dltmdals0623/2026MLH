import time
import serial

ser_enabled = False

try:
    ser = serial.Serial(port="COM3",
                        baudrate=57600,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,   
                        bytesize=serial.EIGHTBITS,
                        timeout=1)
    ser_enabled = True
except serial.SerialException as e:
    ser_enabled = False



#메시지 강조 함수
def stress(msg,nl,color='\033[0m'):
    if nl == 0:
        print()
    print(color+"="*100)
    print("{:^100}".format(msg))
    print("="*100+'\033[0m')
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



def wait_ack(expected_ack=65535):
    if not ser_enabled:
        return False
    packet = ser.read(6)
    if len(packet) < 6:
        return False
    low_byte =packet[2]
    high_byte=packet[4]
    # _, _, low_byte, _, high_byte, _ = packet

    # if h1 != 0xFF or h2 != 0x55:
    #     return False

    received_val = (high_byte << 8) | low_byte
    return received_val == expected_ack



def packet_write(*args):
    if not ser_enabled:
        return [0, 0, 0]

    ser.reset_input_buffer()

    serial_raw(65535)
    if not wait_ack(65535):
        return [-2, 0, 0]

    for key, data in args:
        # ser.reset_input_buffer()
        serial_write(key, data)
        # if not wait_ack(val):
        #     return [-1, key, data]
    # ser.reset_input_buffer()
    serial_raw(65535)
    if not wait_ack(65535):
        return [-3, 0, 0]

    return [1, 0, 0]


def run():
    if not ser_enabled:
        stress(f"COULD NOT OPENS COM3!",1)
        return

    stress("TRANSMIT START!",2, '\033[32m')

    try:
        fail=[]
        total_start_time = time.time()
        for i in range(100):
            pk_ret, pk_key, pk_data = packet_write(*[(n%31, i) for n in range(0, 100)])
            if pk_ret != 1:
                # match(pk_ret):
                #     case 0:
                #         print("SERIAL NOT AVAILABLE                 ",end='')
                #     # case -1:
                #         print(f"Packet [{pk_key}:{pk_data}] 그저 Goa",end='')
                #     case -2:
                #         print("Packet [START] ACK Timeout!          ",end='')
                #     case -3:
                #         print("Packet [END] ACK Timeout!            ",end='')
                print('\033[31m'+'▮',end='',flush=True)
                fail.append(i)
            else:
                print('\033[0m'+'▮',end='',flush=True)
            # print('\r>'+"{:^99}".format(f"Packet [{i+1}/100] SUCCEED!"),end='')
        T=time.time()-total_start_time
        print('\r',end=''*100)
        for i in range(100):
            if i in fail:
                print('\033[31m'+'▮',end='',flush=True)
            else:
                print('\033[32m'+'▮',end='',flush=True)


        stress(f"TOTAL : {time.time() - total_start_time:.3f}sec | PPS : {100/T:.3f}packet | DPS : {10000/T:.3f}data",0,'\033[32m')

    except KeyboardInterrupt:
        print()
        stress("Terminated by USSR",0,'\033[31m')
    finally:
        if ser.is_open:
            ser.close()


if __name__ == "__main__":
    run()