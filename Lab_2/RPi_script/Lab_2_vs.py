#!usr/bin/env python3
""" Импорт библиотек """
import RPi.GPIO as GPIO
from random import randint
from math import exp, sin, cos, sqrt
import threading
import time

import serial
import struct

""" Инициализация GPIO """
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
""" Кнопка подключена к пину №18 """
BUT_PIN = 18
GPIO.setup(BUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

""" Переменные, используемые в переходном процессе """
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

""" Флаг, отвечающий за состояние потока """
thread_alive = 1

""" Флаги, отвечающие за состояние кнопки """
button_state = 0
button_flag = 0

""" Поток для наиболее точного приращения времени """


def timer():
    global t
    global tp
    global thread_alive
    while True:
        if not thread_alive:
            break
        t += 0.01
        tp += 0.04
        time.sleep(0.01)


""" Инициализация потока """
th1 = threading.Thread(target=timer)

""" Подобно коду в C языках """


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
    """ Подключение к USB порту """
    try:
        adress = '/dev/ttyACM0'
        ser = serial.Serial(adress)
    except FileNotFoundError:
        print(f'Can`t connect to {adress}')
    """ Запуск потока """
    th1.start()

    """ Бесконечный цикл """
    while True:
        """ Обработка нажатия кнопки """
        button_state = GPIO.input(BUT_PIN)
        # print(f'BUTTON_STATUS = {button_state}')

        if button_state and not button_flag:
            button_state += 1
            button_state %= 2
            r = randint(0, 3)
            tp = 0
        """ Создание возмущения """
        if r == 0:
            Fv = 0.1 * (1.0206 * exp(-0.2 * tp) * sin(0.9798 * tp))
        elif r == 1:
            Fv = 0.3 * (1.0206 * exp(-0.2 * tp) * sin(0.9798 * tp))
        elif r == 2:
            Fv = 0.55 * (1.0206 * exp(-0.2 * tp) * sin(0.9798 * tp))
        elif r == 3:
            Fv = 0.7 * (1.0206 * exp(-0.2 * tp) * sin(0.9798 * tp))
        # print(f'Fv = {Fv}')

        """ Расчёт значений переходного процесса колебательного звена """
        f1 = sqrt((4 * pow(T2, 2) - pow(T1, 2)) / (4 * pow(T2, 4)))
        f2 = -T1 * k * exp(-T1 * t / (2 * pow(T2, 2))) * sin(t * f1)
        f3 = 2 * pow(T2, 2) * k * f1
        f4 = - 2 * pow(T2, 2) * k * f1 * exp(-T1 * t / (2 * pow(T2, 2))) * cos(t * f1)
        f5 = 2 * pow(T2, 2) * f1
        # print(f1, f2, f3, f4, f5)
        F = (f2 + f3 + f4) / f5 + Fv
        ppp = int(round(F * 10, 0))

        """ Отправка данных в USB """
        ser.write(struct.pack('B', ppp))
        print(ppp)

        button_flag = button_state

    thread_alive = 0


if __name__ == '__main__':
    main()
