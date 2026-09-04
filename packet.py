import os
import time
import queue
import serial
import threading

ser_enabled = False
succeed=[]
fail=[]



try:
    if os.name == 'nt':
        ser = serial.Serial(port="COM4",
                            baudrate=57600,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            timeout=1)
    else:
        ser = serial.Serial(port="/dev/ttyUSB0",
                    baudrate=57600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=1)
    ser_enabled = True
except serial.SerialException as e:
    ser=None
    ser_enabled = False


tx_queue = queue.Queue()

def _tx_worker():
    while True:
        args = tx_queue.get()
        packet_write(*args)
        tx_queue.task_done()

threading.Thread(target=_tx_worker, daemon=True).start()


def retry_serial():
    global ser, ser_enabled
    try:
        if os.name == 'nt':
            ser = serial.Serial(port="COM4",
                                baudrate=57600,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                bytesize=serial.EIGHTBITS,
                                timeout=1)
        else:
            ser = serial.Serial(port="/dev/ttyUSB0",
                        baudrate=57600,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS,
                        timeout=1)
        ser_enabled = True
    except serial.SerialException as e:
        ser=None
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
    if val is None or ser_enabled == False:
        return 0
    down_byte = val & 0xFF
    up_byte = (val >> 8) & 0xFF
    try:
        ser.write(bytearray([255, 85, down_byte, 255-down_byte, up_byte, 255-up_byte]))
    except serial.SerialException as e:
        print(f"SerialError: {str(e)}")
        return 0

    return val



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

    received_val = (high_byte << 8) | low_byte
    return received_val == expected_ack



def packet_write(*args):
    if not ser_enabled:
        retry_serial()
        return 0

    ser.reset_input_buffer()
    serial_raw(65535)
    if not wait_ack(65535):
        return -2

    for key, data in args:
        serial_write(key, data)

    ser.reset_input_buffer()
    serial_raw(65535)
    if not wait_ack(65535):
        return -3

    return 1


def run():
    if not ser_enabled:
        stress(f"COULD NOT OPENS COM4!",1)
        return

    stress("TRANSMIT START!",2, '\033[32m')

    try:
        total_start_time = time.time()
        print('\r',end='▯'*100)
        for i in range(100):
            pk_ret = packet_write(*[(n%31, i) for n in range(0, 100)])
            if pk_ret != 1:
                fail.append(i)
            else:
                succeed.append(i)
            print('\r'+''*i,end='',flush=True)
            for j in range(i+1):
                if j in fail:
                    print('\033[31m'+'▮',end='',flush=True)
                elif j in succeed:
                    print('\033[0m'+'▮',end='',flush=True)
        T=time.time()-total_start_time

        print('\r'+''*100,end='',flush=True)
        for i in range(100):
            if i in fail:
                print('\033[31m'+'▮',end='',flush=True)
            elif i in succeed:
                print('\033[32m'+'▮',end='',flush=True)
            else:
                print('\033[0m'+'▯',end='',flush=True)

        print()
        stress(f"TOTAL : {time.time() - total_start_time:.3f}sec | PPS : {100/T:.3f}packet | DPS : {10000/T:.3f}data | FAIL : {len(fail)}",0,'\033[32m')

    except KeyboardInterrupt:
        print('\r',end=''*100)
        for i in range(100):
                    if i in fail:
                        print('\033[31m'+'▮',end='',flush=True)
                    elif i in succeed:
                        print('\033[32m'+'▮',end='',flush=True)
                    else:
                        print('\033[0m'+'▯',end='',flush=True)
        
        print()
        stress("Interrupted by USSR",0,'\033[31m')
    finally:
        if ser.is_open:
            ser.close()


if __name__ == "__main__":
    run()
