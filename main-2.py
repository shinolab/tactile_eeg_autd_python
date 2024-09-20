import time
import os
import numpy as np
import csv
import random
from pyautd3 import AUTD3, ConfigureSilencer, Controller,GainSTM,FocusSTM,EmitIntensity,LoopBehavior,Segment,ControlPoint
from pyautd3.gain import Focus,Null
from pyautd3.link.soem import SOEM, Status
from pyautd3.modulation import Sine,Static
from pyautd3.driver.geometry import EulerAngles, rad
from pyautd3.gain.holo import GSPAT, EmissionConstraint, pascal, NalgebraBackend
import random
import cv2


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
                .add_device(AUTD3([AUTD3.device_width()+1,0.0,0.0]))
                .add_device(AUTD3([0.0,AUTD3.device_height(),0.0]))
                .add_device(AUTD3([AUTD3.device_width()+1,AUTD3.device_height(),0.0]))
                .open(SOEM.builder().with_err_handler(err_handler),))
autd.send(ConfigureSilencer.disable())

m_100 = Sine(100)
m_150 = Sine(150)
m_200 = Sine(200)
m_s = Static()
m = Static()

center = np.array([AUTD3.device_width() -10.0, -10.0, 240.0])
center = np.array([AUTD3.device_width()/2,AUTD3.device_width()/2,255])

g = Focus( center).with_intensity(255).with_segment(Segment.S1, update_segment=True)
g = Focus(autd.geometry.center+np.array([0,0,255])).with_intensity(255).with_segment(Segment.S1, update_segment=True)
g_off = Focus(center).with_intensity(0).with_segment(Segment.S1, update_segment=True)

name = input("日付を入力してください。\n >>> ")
base_path = r"./" + name + "/"
if not os.path.exists(base_path):
    os.makedirs(base_path)

filepath = r"./" + name + "/"+ "実験2.csv"



trial_num =1 #3
trials = [0]*trial_num + [1]*trial_num + [2]*trial_num + [3]*trial_num + [4]*trial_num + [5]*trial_num
random.shuffle(trials)

trials2 = [0]*trial_num + [1]*trial_num + [2]*trial_num + [3]*trial_num + [4]*trial_num + [5]*trial_num
random.shuffle(trials2)

answers = []

img = cv2.imread("./exp_start.jpg", cv2.IMREAD_COLOR)
cv2.imshow("Exp_window", img)
cv2.waitKey(500)

_ = input()

for i, idx in enumerate(trials):
    print("starting in 3 seconds")
    img = cv2.imread("./exp_start.jpg", cv2.IMREAD_COLOR)
    cv2.imshow("Exp_window", img)
    cv2.waitKey(500)
    time.sleep(2.5)
    if idx==0:
        autd.send((m_100, g))
    elif idx==1:
        autd.send((m_200, g))
    elif idx==2:
        radius = 7.5
        points_num = int(12*7.5)
        stm = FocusSTM.from_freq(5).add_foci_from_iter([
                autd.geometry.center + np.array([radius*np.cos(2*np.pi*i/points_num), radius*np.sin(2*np.pi*i/points_num), 255])
                for i in range(points_num) #一方向に二回
            ]).with_loop_behavior(LoopBehavior.infinite()).with_segment(Segment.S1, update_segment=True)

        autd.send(m_s, stm) 

    elif idx==3:
        radius = 15.0
        points_num = int(12*15)
        stm = FocusSTM.from_freq(5).add_foci_from_iter([
                autd.geometry.center + np.array([radius*np.cos(2*np.pi*i/points_num), radius*np.sin(2*np.pi*i/points_num), 255])
                for i in range(points_num) #一方向に二回
            ]).with_loop_behavior(LoopBehavior.infinite()).with_segment(Segment.S1, update_segment=True)
        
        autd.send(m_s, stm) 

    elif idx==4:
        radius = 7.5
        points_num = int(12*7.5)
        stm = FocusSTM.from_freq(30).add_foci_from_iter([
                autd.geometry.center + np.array([radius*np.cos(2*np.pi*i/points_num), radius*np.sin(2*np.pi*i/points_num), 255])
                for i in range(points_num) #一方向に二回
            ]).with_loop_behavior(LoopBehavior.infinite()).with_segment(Segment.S1, update_segment=True)     
    
        autd.send(m_s, stm) 
    elif idx==5:
        radius = 15
        points_num = int(12*15)
        stm = FocusSTM.from_freq(30).add_foci_from_iter([
                autd.geometry.center + np.array([radius*np.cos(2*np.pi*i/points_num), radius*np.sin(2*np.pi*i/points_num), 255])
                for i in range(points_num) #一方向に二回
            ]).with_loop_behavior(LoopBehavior.infinite()).with_segment(Segment.S1, update_segment=True)       
        autd.send(m_s, stm) 
    time.sleep(0.5)
    null = Null().with_segment(Segment.S0)
    autd.send((m, null))
    print(str(idx)+"の評価")
    while True:
        img = cv2.imread("./fineroughness.jpg", cv2.IMREAD_COLOR)
        cv2.imshow("Exp_window", img)
        cv2.waitKey(500)
        fine_r = input("ザラザラ感\n >>> ")
        if fine_r in ["1","2","3","4","5","6","7"]: 
            break
        else: print("1から7の間の数字を入力せよ")   

    while True:
        img = cv2.imread("./macroroughness.jpg", cv2.IMREAD_COLOR)
        cv2.imshow("Exp_window", img)
        cv2.waitKey(500)
        macro_r = input("凸凹感\n >>> ")
        if macro_r in ["1","2","3","4","5","6","7"]: break
        else: print("1から7の間の数字を入力せよ")   

    while True:
        img = cv2.imread("./hardness.jpg", cv2.IMREAD_COLOR)
        cv2.imshow("Exp_window", img)
        cv2.waitKey(500)
        hardness = input("硬さ\n >>> ")
        if hardness in ["1","2","3","4","5","6","7"]: break
        else: print("1から7の間の数字を入力せよ")   
    #cv2.destroyWindow("Exp_window")
    trial_answer = []
    trial_answer.append(fine_r)
    trial_answer.append(macro_r)
    trial_answer.append(hardness)

    answers.append(trial_answer)

paired_lists = list(zip(trials, answers))
sorted_paired_lists = sorted(paired_lists, key=lambda x: x[0])
sorted_list1, sorted_list2 = zip(*sorted_paired_lists)
sorted_list1 = list(sorted_list1)
sorted_list2 = list(sorted_list2)


answers2 = []
for i, idx in enumerate(trials2):
    print("starting in 3 seconds")
    img = cv2.imread("./exp_start.jpg", cv2.IMREAD_COLOR)
    cv2.imshow("Exp_window", img)
    cv2.waitKey(500)
    time.sleep(2.5)
    if idx==0:
        autd.send((m_100, g))
    elif idx==1:
        autd.send((m_200, g))
    elif idx==2:
        radius = 7.5
        points_num = int(12*7.5)
        stm = FocusSTM.from_freq(5).add_foci_from_iter([
                autd.geometry.center + np.array([radius*np.cos(2*np.pi*i/points_num), radius*np.sin(2*np.pi*i/points_num), 255])
                for i in range(points_num) #一方向に二回
            ]).with_loop_behavior(LoopBehavior.infinite()).with_segment(Segment.S1, update_segment=True)

        autd.send(m_s, stm) 

    elif idx==3:
        radius = 15.0
        points_num = int(12*15)
        stm = FocusSTM.from_freq(5).add_foci_from_iter([
                autd.geometry.center + np.array([radius*np.cos(2*np.pi*i/points_num), radius*np.sin(2*np.pi*i/points_num), 255])
                for i in range(points_num) #一方向に二回
            ]).with_loop_behavior(LoopBehavior.infinite()).with_segment(Segment.S1, update_segment=True)
        
        autd.send(m_s, stm) 

    elif idx==4:
        radius = 7.5
        points_num = int(12*7.5)
        stm = FocusSTM.from_freq(30).add_foci_from_iter([
                autd.geometry.center + np.array([radius*np.cos(2*np.pi*i/points_num), radius*np.sin(2*np.pi*i/points_num), 255])
                for i in range(points_num) #一方向に二回
            ]).with_loop_behavior(LoopBehavior.infinite()).with_segment(Segment.S1, update_segment=True)     
    
        autd.send(m_s, stm) 
    elif idx==5:
        radius = 15
        points_num = int(12*15)
        stm = FocusSTM.from_freq(30).add_foci_from_iter([
                autd.geometry.center + np.array([radius*np.cos(2*np.pi*i/points_num), radius*np.sin(2*np.pi*i/points_num), 255])
                for i in range(points_num) #一方向に二回
            ]).with_loop_behavior(LoopBehavior.infinite()).with_segment(Segment.S1, update_segment=True)       
        autd.send(m_s, stm) 
    time.sleep(0.5)
    null = Null().with_segment(Segment.S0)
    autd.send((m, null))
    print(str(idx)+"の評価")
    while True:
        img = cv2.imread("./largeness.jpg", cv2.IMREAD_COLOR)
        cv2.imshow("Exp_window", img)
        cv2.waitKey(500)
        largeness = input("広さ\n >>> ")
        if largeness in ["1","2","3","4","5","6","7"]: break
        else: print("1から7の間の数字を入力せよ")   

    while True:
        img = cv2.imread("./pleseantness.jpg", cv2.IMREAD_COLOR)
        cv2.imshow("Exp_window", img)
        cv2.waitKey(500)
        pleasentness = input("心地よさ\n >>> ")
        if pleasentness in ["1","2","3","4","5","6","7"]: break
        else: print("1から7の間の数字を入力せよ")   

    while True:
        img = cv2.imread("./strength.jpg", cv2.IMREAD_COLOR)
        cv2.imshow("Exp_window", img)
        cv2.waitKey(500)
        strength = input("強さ\n >>> ")
        if strength in ["1","2","3","4","5","6","7"]: break
        else: print("1から7の間の数字を入力せよ")   
    #cv2.destroyWindow("Exp_window")
    trial_answer2 = []
    trial_answer2.append(largeness)
    trial_answer2.append(pleasentness)
    trial_answer2.append(strength)

    answers2.append(trial_answer2)
cv2.destroyWindow("Exp_window")
paired_lists2 = list(zip(trials2, answers2))
sorted_paired_lists2 = sorted(paired_lists2, key=lambda x: x[0])
sorted_list3, sorted_list4 = zip(*sorted_paired_lists2)
sorted_list3 = list(sorted_list3)
sorted_list4 = list(sorted_list4)

with open(filepath, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([])
    writer.writerow(['刺激種類', '滑らか/ザラザラ', '平ら/凸凹', '柔らかい/硬い', '狭い/広い', '不快/快','弱い/強い'])
    int_lists = []
    for i, value in enumerate(sorted_list1):
        row = [str(value)]+sorted_list2[i]+sorted_list4[i]
        writer.writerow(row)

        int_list = list(map(int, sorted_list2[i]+sorted_list4[i]))
        int_lists.append(int_list)
    arr = np.array(int_lists)

    av = arr.reshape(6, trial_num, 6).mean(axis=1)

    writer.writerow([])
    for j,av_arr in enumerate(av):
        av_row = [j] + av_arr.tolist()
        writer.writerow(av_row)