#!usr/bin/env python3
import RPi.GPIO as GPIO
from random import randint
from math import exp, sin, cos, sqrt
import threading
import time

import serial
import struct

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
BUT_PIN = 18
GPIO.setup(BUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

k = 1
r = 10
T2 = 0.196
T1 = 0.077
t = 0
tp = 0
F = 0
Fv = 0
state = 0
ppp = 0
thread_alive = 1

button_state = 0
button_flag = 0


def timer():
    global t
    global tp
    global thread_alive
    while True:
        if not thread_alive:
            break
        t += 0.01
        tp += 0.01
        time.sleep(0.01)


th1 = threading.Thread(target=timer)


def main():
    time.sleep(5)

    global k
    global r
    global T2
    global T1
    global t
    global tp
    global F
    global Fv
    global ppp
    global button_state
    global button_flag
    global thread_alive

    try:
        adress = '/dev/ttyACM0'
        ser = serial.Serial(adress)
    except FileNotFoundError:
        print(f'Can`t connect to {adress}')

    th1.start()

    while True:

        button_state = GPIO.input(BUT_PIN)
        # print(f'BUTTON_STATUS = {button_state}')

        if button_state and not button_flag:
            button_state += 1
            button_state %= 2
            r = randint(0, 3)
            tp = 0

        if r == 0:
            Fv = 0.1 * (1.0206 * exp(-0.2 * tp) * sin(0.9798 * tp))
        elif r == 1:
            Fv = 0.3 * (1.0206 * exp(-0.2 * tp) * sin(0.9798 * tp))
        elif r == 2:
            Fv = 0.55 * (1.0206 * exp(-0.2 * tp) * sin(0.9798 * tp))
        elif r == 3:
            Fv = 0.7 * (1.0206 * exp(-0.2 * tp) * sin(0.9798 * tp))
        # print(f'Fv = {Fv}')

        f1 = sqrt((4 * pow(T2, 2) - pow(T1, 2)) / (4 * pow(T2, 4)))
        f2 = -T1 * k * exp(-T1 * t / (2 * pow(T2, 2))) * sin(t * f1)
        f3 = 2 * pow(T2, 2) * k * f1
        f4 = - 2 * pow(T2, 2) * k * f1 * exp(-T1 * t / (2 * pow(T2, 2))) * cos(t * f1)
        f5 = 2 * pow(T2, 2) * f1
        # print(f1, f2, f3, f4, f5)
        F = (f2 + f3 + f4) / f5 + Fv
        ppp = int(round(F * 10, 0))

        ser.write(struct.pack('B', ppp))
        print(ppp)

        button_flag = button_state

    thread_alive = 0


if __name__ == '__main__':
    main()
