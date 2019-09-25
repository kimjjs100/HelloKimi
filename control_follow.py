# All code licensed by "Alpha-Car"
# These codes only for Comment

from __future__ import division
# 정수(integer)끼리 나눗셈하면, quotient만 남기고 remainder는 버리는데,
# 해당 코드를 추가하면, 나눗셈연산자(/, //)가 바뀌면서 결과를 float로 표현하게됨(Remainder를 포함하게됨)
import Adafruit_PCA9685
# PCA9685모듈을 임포트
# sudo pip3 install adafruit-pca9685 //파이썬3를 사용할 때
import RPi.GPIO as GPIO
# 라즈베리파이 RPi.GPIO 모듈(for 센서제어) : https://blog.naver.com/pk3152/221368513358
# GPIO는 General-purpose input/output 를 말함.
# pinMode()로 입력 또는 출력으로 설정한 후,
# digitalRead()함수 또는 digitalWrite()함수로 입력이나 출력을 하는 핀을 말함
import socket, threading, time

ip = '192.168.43.160'

Encode = {'A' : 370, 'B' : 370, 'C' : 280, 'D' : 280, 'E' : 30}
        #    car    /   person /   red    /   green  /  stop_sign

pwm = Adafruit_PCA9685.PCA9685()
# pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)
# PWM(Pulse Width Modulation)은 디지털 출력핀인데 아날로그 출력이라는 이름으로 사용

pwm.set_pwm_freq(50)
# 서보모터에 최적화된 Hz로 펄스주기를 설정.
# 해당 프로젝트에서의 서브모터는 50Hz가 최적값인듯.

front_left = 12
front_right = 13
back_left = 15
back_right = 11
# 핀 번호

GPIO.setmode(GPIO.BOARD)
# 핀 번호를 참조하는 방식에 대한 함수는 크게 2가지가 있음.
# GPIO.setmode(GPIO.BOARD) 와 GPIO.setmode(GPIO.BCM)
# BOARD 모드 - 핀 번호를 라즈베리파이의 보드 번호를 참조해서 사용
# BCM 모드 - 핀 번호를 GPIO모듈 번호로 사용
# 자세한 정보는 블로그 참조요망: https://blog.naver.com/pk3152/221368513358

GPIO.setup(front_left, GPIO.OUT)
GPIO.setup(front_right, GPIO.OUT)
GPIO.setup(back_left, GPIO.OUT)
GPIO.setup(back_right, GPIO.OUT)
# 출력핀 설정

CAR_CENTER = 405
CENTER = 307
center = CENTER
SPEED = 1300
# 서보모터의 펄스 길이를 최소, 중간, 최대로 설정
# pulse length out of 4096


CUR_SPEED = SPEED
pwm.set_pwm(0, 0, CENTER)
# pwm.set_pwm(channel, 0, pulse)
# 0번 서보를 CENTER(307)으로 설정

def straight(pwm=pwm, GPIO=GPIO, speed=CUR_SPEED, center=CENTER):
# 직진에 대한 정의
    pwm.set_pwm(0, 0, center)
    pwm.set_pwm(4, 0, speed)
    pwm.set_pwm(5, 0, speed)
    # 4,5번 서보모터를 speed(1300)으로 설정
    GPIO.output(front_left, GPIO.HIGH)
    GPIO.output(front_right, GPIO.HIGH)
    # 앞 바퀴 2개 모두 직진.
    # < 출력핀에 5V를 내보낼 때, 2가지 설정 방법 (택1) >
    # GPIO.output(pin번호,GPIO.HIGH)
    # GPIO.output(pin번호,True)

    GPIO.output(back_left, GPIO.LOW)
    GPIO.output(back_right, GPIO.LOW)
    # 뒷 바퀴 2개 모두 출력신호 없음(전륜구동인듯).
    # < 출력핀에 0V를 내보낼 때, 2가지 설정 방법 (택1) >
    # GPIO.output(pin번호,GPIO.LOW)
    # GPIO.output(pin번호,False)

def stop(pwm=pwm, GPIO=GPIO, speed=CUR_SPEED):
# 정지에 대한 정의
    pwm.set_pwm(4, 0, speed)
    pwm.set_pwm(5, 0, speed)
    GPIO.output(front_left, GPIO.HIGH)
    GPIO.output(front_right, GPIO.HIGH)
    GPIO.output(back_left, GPIO.LOW)
    GPIO.output(back_right, GPIO.LOW)

def left(pwm=pwm, GPIO=GPIO, speed=CUR_SPEED, center=CENTER, turn=100):
# 좌회전에 대한 정의
    pwm.set_pwm(0, 0, center + turn)  # 410
    pwm.set_pwm(4, 0, speed)
    pwm.set_pwm(5, 0, speed)
    GPIO.output(front_left, GPIO.HIGH)
    GPIO.output(front_right, GPIO.HIGH)
    GPIO.output(back_left, GPIO.LOW)
    GPIO.output(back_right, GPIO.LOW)

def right(pwm=pwm, GPIO=GPIO, speed=CUR_SPEED, center=CENTER, turn=100):
#우회전에 대한 정의
    pwm.set_pwm(0, 0, center - turn)  # 210
    pwm.set_pwm(4, 0, speed)
    pwm.set_pwm(5, 0, speed)
    GPIO.output(front_left, GPIO.HIGH)
    GPIO.output(front_right, GPIO.HIGH)
    GPIO.output(back_left, GPIO.LOW)
    GPIO.output(back_right, GPIO.LOW)

Object = False
Bus_stop = False
Obstacle = False
red_light = False
# 해당 시그널을 확인하면 멈춤


def from_line():
    global Object, Bus_stop, Obstacle, red_light
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (ip, 3442)
    print("line socket listening...")
    sock.bind(server_address)
    sock.listen(1)

    try:
        client, address = sock.accept()
        print("Line Connected")
        while True:
            data = client.recv(4)
            if Object or Bus_stop or Obstacle or red_light:
                #print("stop")
                stop(speed=0)
                continue
            try:
                x_point = int(data)
            except:
                continue
            if x_point >= CAR_CENTER - 50 and x_point <= CAR_CENTER + 50:
                #print("straight : ", x_point)
                straight()
            elif CAR_CENTER >= x_point:
                k = int((CAR_CENTER - x_point) / 4)
                #print("left : ", x_point," -> ",k + CENTER)
                right(speed=CUR_SPEED + 100, turn=k)
            elif CAR_CENTER < x_point:
                k = int((x_point - CAR_CENTER) / 4)
                #print("right : ", x_point, " -> ", k + CENTER)
                left(speed=CUR_SPEED + 100, turn=k)
    except:
        print("close line")
        GPIO.cleanup()
        exit(0)


def from_YOLO():
    global Object, Bus_stop, Obstacle, CUR_SPEED, red_light
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (ip, 3443)
    print("YOLO socket listening...")
    sock.bind(server_address)
    sock.listen(1)
    try:
        client, address = sock.accept()
        print("YOLO connected")
        while True:
            data = client.recv(4)
            if data == 'gogo':
                if not Bus_stop and not Obstacle:
                    print("Object : GO")
                    stop(speed=SPEED)
                    Object = False
                    red_light = False
                continue

            Type, ymax = data[0], int(data[1:])
            if Type == 0 or Type == None or Type == '0':
                continue

            if Encode[Type] < ymax:
                stop(speed=0)
                if Type == 'C':
                    red_light = True
                elif Type == 'D':
                    red_light = False
                else:
                    Object = True
                print("Object : Stop!!  ",Type)
            elif Encode[Type] - 80 >= ymax:
                CUR_SPEED = SPEED
                if Type == 'C':
                    red_light = False
                else:
                    Object = False
                print("Object : enough distance")
            else:
                CUR_SPEED = SPEED-200
                if Type == 'C':
                    red_light = False
                else:
                    Object = False
                #CUR_SPEED = Encode[Type] - ymax + 1200  #1100 is min-speed
                print("Object : Slow Down ",CUR_SPEED)

    except:
        print("close yolo")
        sock.close()
        GPIO.cleanup()
        exit(1)

def from_RFID():
    global Bus_stop, Obstacle
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (ip, 3444)
    print("Sensor socket listening...")
    sock.bind(server_address)
    sock.listen(1)

    try:
        client, address = sock.accept()
        while True:
            data = client.recv(4)
            if data == "gogo" and not Obstacle:
                stop(speed=SPEED)
                Bus_stop = False
            elif data == "stop":
                stop(speed=0)
                Bus_stop = True
    except:
        print("close RFID")
        sock.close()
        GPIO.cleanup()
        exit(1)

def from_Ultra():
    global Obstacle, Bus_stop
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (ip, 3445)
    print("Sensor socket listening...")
    sock.bind(server_address)
    sock.listen(1)

    try:
        client, address = sock.accept()
        while True:
            data = client.recv(4)
            if data == "gogo" and not Bus_stop:
                stop(speed=SPEED)
                Obstacle = False
            elif data == "stop":
                stop(speed=0)
                Obstacle = True
    except:
        print("close ultra")
        sock.close()
        GPIO.cleanup()
        exit(1)

LINE = threading.Thread(target=from_line)
DETECT = threading.Thread(target=from_YOLO)
RFID = threading.Thread(target=from_RFID)
ULTRA = threading.Thread(target=from_Ultra)

LINE.start()
DETECT.start()
RFID.start()
ULTRA.start()

LINE.join()
DETECT.join()
RFID.join()
ULTRA.join()

pwm.set_pwm(0, 0, 0)
GPIO.cleanup()
