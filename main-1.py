import os
import numpy as np
from pyautd3 import AUTD3, ConfigureSilencer, Controller,GainSTM,FocusSTM,EmitIntensity,LoopBehavior,Segment,ControlPoint
from pyautd3.gain import Focus,Null
from pyautd3.link.soem import SOEM, Status
from pyautd3.modulation import Sine,Static
from pyautd3.driver.geometry import EulerAngles, rad
from pyautd3.gain.holo import GSPAT, EmissionConstraint, pascal, NalgebraBackend
import random

import time
import serial
import datetime
import threading
from pynput.mouse import Listener


def on_click(x, y, button, pressed):
    if pressed:
        print(f'クリックされました:')
        # ファイルを開く
        with open(file_path, 'a') as file:
            write_timestamp(file)

def start_mouse_listener():
    # マウスクリックのリスナーを作成
    with Listener(on_click=on_click) as listener:
        listener.start()
listener = Listener(on_click=on_click)

# メインスレッドの処理
def main_thread_process():
    while True:
        # ここにメインスレッドで行いたい処理を書く
        pass

def write_timestamp(file):
    now = datetime.datetime.now()
    formatted_time = now.strftime("%Y-%m-%d %H:%M:%S")
    file.write(formatted_time + "\n")

def err_handler(slave: int, status: Status, msg: str) -> None:
        match status:
            case Status.Error:
                print(f"Error [{slave}]: {msg}")
            case Status.Lost:
                print(f"Lost [{slave}]: {msg}")
                os._exit(-1)
            case Status.StateChanged:
                print(f"StateChanged  [{slave}]: {msg}")


autd = (Controller.builder()
                .add_device(AUTD3([0.0, 0.0, 0.0]))
                .add_device(AUTD3([AUTD3.device_width(),0.0,0.0]))
                .add_device(AUTD3([0.0,AUTD3.device_height(),0.0]))
                .add_device(AUTD3([AUTD3.device_width(),AUTD3.device_height(),0.0]))
                .open(SOEM.builder().with_err_handler(err_handler),))
autd.send(ConfigureSilencer.disable())

m_100 = Sine(100)
m_150 = Sine(150)
m_200 = Sine(200)
m_s = Static()
m = Static()

center = autd.geometry.center + np.array([0,0, 255])
g = Focus(center).with_intensity(255)
g_off = Focus(center).with_intensity(0)


# COMポートの設定
ser = serial.Serial('COM7', 115200, timeout=1, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE)

if ser.isOpen():
    print("COM3 port found.")
else:
    print("Failed to open COM3 port.")
    exit()

# トリガーの用意
dataToSendRR = b"RR"  # バイト列として定義する
dataToSend0 = b"00"
dataToSend1 = b"01"
dataToSend2 = b"02"
dataToSend3 = b"03"
dataToSend4 = b"04"
dataToSend5 = b"05"
dataToSend6 = b"06"
dataToSend7 = b"07"
dataToSend8 = b"08"
bytesWritten = 0

# BrainAmpではRRを2回送る
ser.write(dataToSendRR)
ser.write(dataToSendRR)


# メルセンヌ・ツイスター乱数生成器を初期化
n = random.randrange(10)
random.seed(n)

# trials配列を作成し、0から6までの値を規則的に埋め、その後シャッフルする
trial_num =1 #25
trial_num2 =1 #10
trials = [0]*trial_num + [1]*trial_num + [2]*trial_num + [3]*trial_num + [4]*trial_num + [5]*trial_num + [6]*trial_num2
random.shuffle(trials)
stim_time = 0.5 #0.5


name = input("日付を入力してください。\n >>> ")
base_path = r"./" + name + "/"
if not os.path.exists(base_path):
    os.makedirs(base_path)
session = input("session番号を入力してください。\n >>> ")

file_path = r"./" + name + "/session"+session + ".csv"
# ファイルを開く
with open(file_path, 'a') as file:
    write_timestamp(file)

# 3秒間の遅延を挿入する
time.sleep(3)

listener.start()

for trial in trials:
    if trial==0:
        print(trial)
        ser.write(dataToSend1)
        autd.send((m_100, g))
        ser.write(dataToSend0)
        time.sleep(stim_time)
        null = Null()
        autd.send((m, null))

    elif trial==1:
        print(trial)
        ser.write(dataToSend2)
        autd.send((m_200, g))
        ser.write(dataToSend0)
        time.sleep(stim_time)
        null = Null()
        autd.send((m, null))
    elif trial==2:
        print(trial)

        radius = 7.5
        points_num = int(12*7.5)
        stm = FocusSTM.from_freq(5).add_foci_from_iter([
                autd.geometry.center + np.array([radius*np.cos(2*np.pi*i/points_num), radius*np.sin(2*np.pi*i/points_num), 255])
                for i in range(points_num) #一方向に二回
            ]).with_loop_behavior(LoopBehavior.infinite()).with_segment(Segment.S1, update_segment=True)
        ser.write(dataToSend3)
        autd.send(m_s, stm) 
        ser.write(dataToSend0)
        time.sleep(stim_time)
        null = Null()
        autd.send((m, null))
    elif trial==3:
        print(trial)

        radius = 15.0
        points_num = int(12*15)
        stm = FocusSTM.from_freq(5).add_foci_from_iter([
                autd.geometry.center + np.array([radius*np.cos(2*np.pi*i/points_num), radius*np.sin(2*np.pi*i/points_num), 255])
                for i in range(points_num) #一方向に二回
            ]).with_loop_behavior(LoopBehavior.infinite()).with_segment(Segment.S1, update_segment=True)

        ser.write(dataToSend4)   
        autd.send(m_s, stm) 
        ser.write(dataToSend0)
        time.sleep(stim_time)
        null = Null()
        autd.send((m, null))
    elif trial==4:
        print(trial)

        radius = 7.5
        points_num = int(12*7.5)
        stm = FocusSTM.from_freq(30).add_foci_from_iter([
                autd.geometry.center + np.array([radius*np.cos(2*np.pi*i/points_num), radius*np.sin(2*np.pi*i/points_num), 255])
                for i in range(points_num) #一方向に二回
            ]).with_loop_behavior(LoopBehavior.infinite()).with_segment(Segment.S1, update_segment=True)
        
        ser.write(dataToSend5)        
        autd.send(m_s, stm)
        ser.write(dataToSend0)
        time.sleep(stim_time)
        null = Null()
        autd.send((m, null))
    elif trial==5:
        print(trial)

        radius = 15
        points_num = int(12*15)
        stm = FocusSTM.from_freq(30).add_foci_from_iter([
                autd.geometry.center + np.array([radius*np.cos(2*np.pi*i/points_num), radius*np.sin(2*np.pi*i/points_num), 255])
                for i in range(points_num) #一方向に二回
            ]).with_loop_behavior(LoopBehavior.infinite()).with_segment(Segment.S1, update_segment=True)

        ser.write(dataToSend6)        
        autd.send(m_s, stm)
        ser.write(dataToSend0)
        time.sleep(stim_time)
        null = Null()
        autd.send((m, null))
    elif trial ==6:
        #mouse_thread = threading.Thread(target=start_mouse_listener)
        #mouse_thread.start()

        print(trial)
        ser.write(dataToSend7)
        time.sleep(0.01)
        ser.write(dataToSend0)
        autd.send(m_100,g)
        time.sleep(0.1)
        null = Null()
        autd.send(m,null)
        time.sleep(0.1)
        autd.send(m_100,g)
        time.sleep(0.1)
        null = Null()
        autd.send(m,null)
    time.sleep(1.0)
    ser.write(dataToSend8)
    time.sleep(0.01)
    ser.write(dataToSend0)

    mt64 = random.Random()

    # 1000ミリ秒（1秒）にランダムな追加時間を加えた時間だけプログラムの実行を一時停止する
    additional_time = int(mt64.random() * 1000)
    time.sleep(1 + additional_time / 1000)

autd.send((m_100, g_off))
time.sleep(0.5)
null = Null()
autd.send((m, null))
listener.stop()

# COMポートを閉じる
ser.close()

autd.close()


