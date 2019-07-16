#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import os
import re
import sys
import cv2
import time
import rospy
import almath
import atexit
import thread
import actionlib
import numpy as np
from threading import Thread
from std_srvs.srv import Empty
from object_detection import waving_detection
from object_detection import darknet_v2
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from tf import transformations
from speech_recog import baidu_recognition_text

class restaurant():
    def __init__(self, params):
        rospy.init_node("restaurant")
        atexit.register(self.__del__)
        self.ip = params["ip"]
        self.port = params["port"]
        self.session = qi.Session()
        try:
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print("[Kamerider E] : connection Error!!")
            sys.exit(1)
        self.Leds = self.session.service("ALLeds")
        self.Memory = self.session.service("ALMemory")
        self.Motion = self.session.service("ALMotion")
        self.VideoDev = self.session.service("ALVideoDevice")
        self.AudioPla = self.session.service("ALAudioPlayer")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.AudioRec = self.session.service("ALAudioRecorder")
        self.SoundDet = self.session.service("ALSoundDetection")
        self.BasicAwa = self.session.service("ALBasicAwareness")
        self.TabletSer = self.session.service("ALTabletService")
        self.AutonomousLife = self.session.service("ALAutonomousLife")

        try:
            self.AudioRec.stopMicrophonesRecording()
        except BaseException:
            print("\033[0;33m\t[Kamerider W] : You don't need stop record\033[0m")
        # 录音的函数
        self.thread_recording = Thread(target=self.record_audio, args=(None,))
        self.thread_recording.daemon = True
        self.audio_terminate = False
        # ROS 订阅器和发布器
        self.nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.nav_as.wait_for_server()
        # YOLO相关
        self.waving_detector = waving_detection.object_detection()
        self.drink_detector=darknet_v2.object_detection()
        #self.detector = dlib.get_frontal_face_detector()
        self.get_image_switch = 1
        # 清除costmap
        self.map_clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.map_clear_srv()
        # 声明一些变量
        #self.IMGPATH=/home/hts/RoboCup2019/object_detection/data/waving_detection.jpg
        self.object = ["coke cole", "wine", "beer", "photo chips", "green tea", "water"]
        self.current_drink_name = []
        self.scan_msg = []
        self.if_save_switch = True
        self.scan_msg_time = 0
        self.angle = -.2
        self.f = open('./laser.txt', 'w')
        self.if_need_record = False
        self.head_fix = True
        self.bar_location = "none"
        self.point_dataset = []
        self.nameId=0
        self.width = 640
        self.height = 480
        self.image = np.zeros((self.height, self.width, 3), np.uint8)
        self.order_num=0
        self.hand_switch=False
        self.order_in_bar=[]
        self.missing_order=[]
        # 设置英语
        self.TextToSpe.setLanguage("English")
        #    LED的group
        self.led_name = ["Face/Led/Blue/Right/0Deg/Actuator/Value", "Face/Led/Blue/Right/45Deg/Actuator/Value",
                         "Face/Led/Blue/Right/90Deg/Actuator/Value", "Face/Led/Blue/Right/135Deg/Actuator/Value",
                         "Face/Led/Blue/Right/180Deg/Actuator/Value", "Face/Led/Blue/Right/225Deg/Actuator/Value",
                         "Face/Led/Blue/Right/270Deg/Actuator/Value", "Face/Led/Blue/Right/315Deg/Actuator/Value",
                         "Face/Led/Blue/Left/0Deg/Actuator/Value", "Face/Led/Blue/Left/45Deg/Actuator/Value",
                         "Face/Led/Blue/Left/90Deg/Actuator/Value", "Face/Led/Blue/Left/135Deg/Actuator/Value",
                         "Face/Led/Blue/Left/180Deg/Actuator/Value", "Face/Led/Blue/Left/225Deg/Actuator/Value",
                         "Face/Led/Blue/Left/270Deg/Actuator/Value", "Face/Led/Blue/Left/315Deg/Actuator/Value"]
        self.Leds.createGroup("MyGroup", self.led_name)
        # 关闭basic_awareness
        if self.BasicAwa.isEnabled():
            self.BasicAwa.setEnabled(False)
        if self.BasicAwa.isRunning():
            self.BasicAwa.pauseAwareness()
        # 初始化平板
        self.TabletSer.cleanWebview()
        print ('\033[0;32m [Kamerider I] Tablet initialize successfully \033[0m')
        # 录下的音频保存的路径
        self.audio_path = '/home/nao/audio/record.wav'
        self.recog_result = "None"
        # Beep 音量
        self.beep_volume = 70
        # 设置初始化头的位置 走路的时候固定头的位置
        self.Motion.setStiffnesses("Head", 1.0)
        self.Motion.setAngles("Head", [0., -0.25], .05)
        # 设置说话速度
        self.TextToSpe.setParameter("speed", 80.0)
        # 关闭AutonomousLife模式
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("Stand", .5)
        # 初始化录音
        self.record_delay = 2
        self.speech_hints = []
        self.enable_speech_recog = True
        self.SoundDet.setParameter("Sensitivity", .8)
        self.SoundDet.subscribe('sd')
        self.SoundDet_s = self.Memory.subscriber("SoundDetected")
        self.SoundDet_s.signal.connect(self.callback_sound_det)
        # 调用成员函数
        self.start_head_fix()
        self.set_volume(.7)

    def __del__(self):
        print ('\033[0;32m [Kamerider I] System Shutting Down... \033[0m')

    def callback_sound_det(self, msg):
        if self.if_need_record:
            print ('\033[0;32m [Kamerider I] Sound detected (In callback function) \033[0m')
            ox = 0
            for i in range(len(msg)):
                if msg[i][1] == 1:
                    ox = 1
            if ox == 1 and self.enable_speech_recog:
                self.record_time = time.time() + self.record_delay
                print "self.record_time += 3s ... ", self.record_time
                #if not self.thread_recording.is_alive():
                #    self.start_recording(reset=True)
                while self.recog_result == "00":
                    time.sleep(1)
                    continue
            else:
                return None
        else:
            print ('\033[0;32m [Kamerider I] Sound detected, but we don\'t need to record this audio \033[0m')

    def set_volume(self, volume):
        self.TextToSpe.setVolume(volume)
    def get_cur_img(self):
        
        get_img=False
        result = self.VideoDev.getImageRemote(self.nameId)
        if result == None:
            print 'cannot capture.'
        elif result[6] == None:
            print 'no image data string.'
        else:
            values = map(ord, list(str(bytearray(result[6]))))
            i = 0
            for y in range(0, self.height):
                for x in range(0, self.width):
                    self.image.itemset((y, x, 0), values[i + 0])
                    self.image.itemset((y, x, 1), values[i + 1])
                    self.image.itemset((y, x, 2), values[i + 2])
                    i += 3
            # print image
            cv2.imshow("waving_detection", self.image)
            cv2.waitKey(5)
            cv2.imwrite("/home/hts/RoboCup2019/object_detection/data/waving_detection.jpg", self.image)
            get_img=True
        return get_img 

    def get_drink_img(self):
        print "@@@@@"
        get_img=False
        result = self.VideoDev.getImageRemote(self.nameId)
        if result == None:
            print 'cannot capture.'
        elif result[6] == None:
            print 'no image data string.'
        else:
            values = map(ord, list(str(bytearray(result[6]))))
            i = 0
            for y in range(0, self.height):
                for x in range(0, self.width):
                    self.image.itemset((y, x, 0), values[i + 0])
                    self.image.itemset((y, x, 1), values[i + 1])
                    self.image.itemset((y, x, 2), values[i + 2])
                    i += 3
            # print image
            print "take a pic"
            cv2.imshow("drink_detection", self.image)
            cv2.waitKey(5)
            cv2.imwrite("/home/hts/RoboCup2019/object_detection/data/drink_detection.jpg", self.image)
            get_img=True
        return get_img       


    def approach_waving(self):
        self.angle = -.2
        # 记录旋转的次数
        turn_num = 2
        self.Motion.setAngles("Head", [0., self.angle], .2)
        AL_kQVGA = 2
        # Need to add All color space variables
        AL_kRGBColorSpace = 13
        fps = 60
        self.nameId = self.VideoDev.subscribe("image" + str(time.time()), AL_kQVGA, AL_kRGBColorSpace, fps)          
        if_turn_finished = False
        face_detect_num = 0
        check_second_time = False
        waving_dete = True
        Need_help=False
        while self.get_image_switch==1:
            self.get_image_switch=2
            while self.get_cur_img()==False:
                print "get_cur_img_false........"
                get_image_switch=1
                continue
            print "get-cur_img_true......"
            #当前是否要检测挥手
            get_new_img=0
            while waving_dete:
                get_new_img+=1
                if get_new_img>40:
                    print"stop get new_img and ask OC for help"
                    self.TextToSpe.say("dear operator I can not find a guest please lead me to his or her table")
                    Need_help=True
                    break                
                print "get_new_img",get_new_img
                self.get_cur_img()
                if self.waving_detector.detector("/home/hts/RoboCup2019/object_detection/data/waving_detection.jpg") == "none":
                    print"--------wave_none------"
                    # 第二次还是没识别到，就转身
                    if check_second_time:
                        check_second_time = False
                        if turn_num != 4:
                            self.Motion.moveTo(0, 0, 0.785)
                            turn_num += 1
                        else:
                            self.Motion.moveTo(0, 0, 3.14)
                            turn_num = 0
                            continue
                    # 如果第一次没识别到，就再识别一次
                    else:
                        check_second_time = True
                        continue
                # 只要识别到了就接近人
                else:
                    waving_dete = False
                    check_second_time = False
                    print"wave_detect_succ"
                    self.waving_detector.main("/home/hts/RoboCup2019/object_detection/data/waving_detection.jpg")
                
            reach_person=False
            get_person_img=0
            while reach_person==False and Need_help==False:
                get_person_img+=1
                print "get_new_per_img",get_person_img
                # if get_person_img>40:
                #     reach_person=True
                self.get_cur_img()
                if self.waving_detector.detector("/home/hts/RoboCup2019/object_detection/data/waving_detection.jpg")!="none":
                    r=self.waving_detector.detector("/home/hts/RoboCup2019/object_detection/data/waving_detection.jpg")
                    print "r:::::::::::",r
                    # max_reliable 
                    reliable_max = 0
                    for i in range(len(r)):
                        print"reliable:",i,r[i][1]
                        if reliable_max<r[i][1]:
                            reliable_max=r[i][1]
                            wave_person=i                    
                        if r[i][1] > 0:
                            name = r[i][0]
                            rects = r[i][2]
                            cv2.rectangle( self.image, (int(rects[0] - rects[2] / 2), int(rects[1] - rects[3] / 2 - 10)),(int(rects[0] + rects[2] / 2), int(rects[1] + rects[3] / 2)), (0, 0, 255), 3)
                            cv2.putText(self.image, name, (int(rects[0] - rects[2] / 2), int(rects[1] - rects[3] / 2)), cv2.FONT_HERSHEY_SIMPLEX,
                                        1, (255, 0, 0), 3)
                    rect = r[wave_person][2]
                    cv2.putText(self.image, name, (int(rect[0] - rect[2] / 2), int(rect[1] - rect[3] / 2)), cv2.FONT_HERSHEY_SIMPLEX,
                                        1, (0, 0, 255), 3)
                    cv2.imshow("approach_person_img", self.image)
                    cv2.waitKey(1)
                    cv2.imwrite("./restaurant_person.jpg", self.image)
                    print "wave_person_ID",wave_person
                    cur_left=int(rect[0] - rect[2] / 2)
                    cur_right=int(rect[0] + rect[2] / 2)
                    cur_top=int(rect[1] - rect[3] / 2 - 10)
                    cur_bottom=int(rect[1] + rect[3] / 2)
                    if if_turn_finished:
                        print "approach.........."
                        height_center = (cur_top + cur_bottom) / 2
                        # 判断是不是要zhuan tou 
                        print "rate:", (float(cur_right-cur_left)*float(cur_bottom-cur_top))/(self.width*self.height)
                        print "height_center error:",abs(height_center - self.height / 2)
                        if (float(cur_right-cur_left)*float(cur_bottom-cur_top))/(self.width*self.height)>0.15:
                            self.set_velocity(0, 0, 0)
                            print "reaching person need to adjust head"
                        if (abs(height_center - self.height / 2)<20) and (float(cur_right-cur_left)*float(cur_bottom-cur_top))/(self.width*self.height)>0.15:
                            print "finish"
                            self.set_velocity(0, 0, 0)
                            reach_person=True
                            break
                        #self.get_image_switch = False
                        elif height_center < self.height / 2:
                            print "upupupupupupupupupupupupupupup", cur_bottom
                            self.angle -= .05
                            self.Motion.setAngles("Head", [0., self.angle], .2)                                  
                        elif height_center > 2 * self.height / 3:
                            print "downdowndowndowndowndowndowndown", cur_top
                            self.angle += .05
                            self.Motion.setAngles("Head", [0., self.angle], .2)
                        # 直走
                        self.set_velocity(0.15, 0, 0)
                        # 判断是否需要微调
                        center = (cur_left + cur_right) / 2
                        if abs(self.width / 2 - center) > self.width / 10:
                            print "inininininininininininnnininininini"
                            Error_dist_ = self.width / 2 - center
                            self.set_velocity(0, 0, 0.001 * Error_dist_)
                            time.sleep(2)
                            self.set_velocity(0.15, 0, 0)
                    if if_turn_finished==False:
                        self.center = (cur_left + cur_right )/ 2
                        Error_dist = self.width / 2 - self.center
                        print "Error_dist",Error_dist
                        if abs(Error_dist) <= 30:
                            if_turn_finished = True
                            #reach_person=False
                            continue
                        self.Motion.moveTo(0, 0, 0.002 * Error_dist)
                        print"turn to person"
                        cv2.waitKey(1)
                # 没有检测到人脸就旋转
                else:

                    self.set_velocity(0,0,0)
                    if (float(cur_right-cur_left)*float(cur_bottom-cur_top))/(self.width*self.height)>0.15:
                        print "cannot detect more and finish"
                        reach_person=True
                        break
                    if face_detect_num != 3:
                        face_detect_num += 1
                        continue
                    else:
                        face_detect_num = 0
                    if turn_num != 4:
                        #self.set_velocity(0,0,0)
                        self.set_velocity(0, 0, 0.3925)
                        time.sleep(2)
                        self.set_velocity(0,0,0)
                        print "finding....turn1"
                        turn_num += 1
                    else:
                        #self.Motion.moveTo(0, 0, -3.14)
                        self.set_velocity(0,0,-1.06)
                        time.sleep(3)
                        self.set_velocity(0,0,0)
                        print "finding@@@@@@@@turn2"
                        turn_num = 0
                

        return "succe"

    def start_recording(self, reset=False, base_duration=3, withBeep=True):
        self.if_need_record = True
        self.record_time = time.time() + base_duration
        if reset:
            self.kill_recording_thread()
            self.AudioRec.stopMicrophonesRecording()
        print "self.record_time is:  ", self.record_time
        if not self.thread_recording.is_alive():
            self.thread_recording = Thread(target=self.record_audio, args=(self.speech_hints, withBeep))
            self.thread_recording.daemon = False
            self.thread_recording.start()
            self.thread_recording.join()
            print('\033[0;32m [Kamerider I] Start recording thread \033[0m')

    def start(self):
        print "start"
        # 检测吧台
        self.scan_sub = rospy.Subscriber('/pepper_robot/naoqi_driver/laser', LaserScan, self.scan_callback)
        while self.bar_location == "none":
            if self.scan_msg_time == 4:
                time_left = time_right = 0
                sum_left = sum_right = 0
                for i in range(4):
                    dis_left=[]
                    dis_right=[]
                    for j in range(len(self.scan_msg[i].ranges)):
                        print "-----------"
                        print j, self.scan_msg[i].ranges[j]
                    for j in range(0,15):
                        dis_right.append(self.scan_msg[i].ranges[j])
                        #sum_right += self.scan_msg[i].ranges[j]
                        # print "===================right", self.scan_msg[i].ranges[j]
                    dis_right.sort(cmp=None, key=None, reverse=False)
                    for k in range(0,4):
                        sum_right+=dis_right[k]
                    for j in range(50, 56):
                        dis_left.append(self.scan_msg[i].ranges[j])
                        #sum_left += self.scan_msg[i].ranges[j]
                        # print "-------------------left", self.scan_msg[i].ranges[j + 31]
                    dis_left.sort(cmp=None, key=None, reverse=False)
                    for k in range(0,4):
                        sum_left+=dis_left[k]
                    if sum_right < sum_left:
                        time_right += 1
                        print "1111111"
                    else:
                        time_left += 1
                        print "0000000"
                if time_right > time_left:
                    self.bar_location = "right"
                    print "right"
                else:
                    self.bar_location = "left"
                    print "left"
        self.TextToSpe.say("The bar table is on the " + self.bar_location)
        #save point 1 bar_table
        self.save_point()
        print "save start point1"
        #save point 2 find person point   
        self.Motion.moveTo(1.5, 0, 0)
        self.save_point()
        print "save start_find point2"
        #detect person start
        self.show_image("please-waving.png")
        self.TextToSpe.say("I am going to find the guest,please waving to me")
        # find guest function 抬头找人
        self.angle = -.2
        print "start find person"
        self.approach_waving()
        print "finish_approach_waving"
        #save point_3 person order point
        self.save_point()
        print "save_point_person3"
        self.show_image("instructions.png")
        self.TextToSpe.say("hello, What can I do for you?")
        self.start_recording(reset=True)
        self.analyze_content()
        self.Motion.moveTo(0, 0, -3.14)
        self.go_to_waypoint(self.point_dataset[0])
        print "go to table bar"
        #move to table bar
        if self.bar_location=="right":
            self.Motion.moveTo(0.5,0,0)
            self.Motion.moveTo(0,0,-1.57)
            self.Motion.moveTo(0.5,0,0)
            self.Motion.moveTo(0,0,-1.57)
        if self.bar_location=="left":
            self.Motion.moveTo(0.5,0,0)
            self.Motion.moveTo(0,0,1.57)
            self.Motion.moveTo(0.5,0,0)
            self.Motion.moveTo(0,0,1.57) 
        #save detect point
        self.save_point()
        print "bar detect point save4"
        self.detect_drinks()
        self.go_to_waypoint(self.point_dataset[2])
        self.Motion.setAngles("Head", [0., -0.18], .05)
        self.dialog_with_people()
        self.go_to_waypoint(self.point_dataset[1])
        print "go to find person point2"
    
    def dialog_with_people(self):
        if len(self.order_in_bar)==0 and len(self.missing_order)!=0:
            self.TextToSpe.say("sorry, the bar table do not have ")
            for drinks in range(0,len(self.missing_order)):
                self.TextToSpe.say(self.missing_order[drinks])
        else:
            self.TextToSpe.say("hello, this is your order")
            if len(self.missing_order)!=0:
                self.TextToSpe.say("and sorry the bar table do not have")
                for i in range (len(self.missing_order)):
                    self.TextToSpe.say(self.missing_order[i])

    def show_image(self, image_name):
        # cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":./person_image/person_image.png ~/.local/share/PackageManager/apps/boot-config/html"
        # os.system(cmd)
        self.TabletSer.hideImage()
        self.TabletSer.showImageNoCache("http://198.18.0.1/apps/boot-config/" + str(image_name))


    def detect_drinks(self):

        AL_kQVGA = 2
        # Need to add All color space variables
        AL_kRGBColorSpace = 13
        fps = 60
        self.nameId = self.VideoDev.subscribe("image" + str(time.time()), AL_kQVGA, AL_kRGBColorSpace, fps)

        
        self.TextToSpe.say("hey the guest need ")
        for i in range(len(self.current_drink_name)):
            self.TextToSpe.say(" " + self.current_drink_name[i])
        # for i in range(len(self.current_drink_name)):
            #self.TextToSpe.say(" " + self.current_drink_name[i])
            #drinks detect
        detect_num_drinks=0
        need_drinks_detect=True
        while need_drinks_detect==True:
            print "detect_num_drinks", detect_num_drinks
            if detect_num_drinks>20:
                need_drinks_detect=False
                self.hand_switch=True
                self.hand_over()
                self.TextToSpe.say("please hand over the drinks in my hand")                    
            detect_num_drinks+=1
            self.get_drink_img() 
            print "get drinks img"
            if True:
            #if self.drink_detector.detector("/home/hts/RoboCup2019/object_detection/data/drink_detection.jpg")!="none":
                need_drinks_detect=False
                #drinks=self.drink_detector.detector("/home/hts/RoboCup2019/object_detection/data/waving_detection.jpg")

                # drink detected picture to pepper
                # cmd = "sshpass -p kurakura326 scp /home/hts/RoboCup2019/object_detection/data/drink_detection.jpg nao@" + str(
                # self.ip) + ":~/.local/share/PackageManager/apps/boot-config/html"
                # os.system(cmd)
                # self.show_image("drink_detection.jpg")
                detect_drink=["green tea"]
                self.TextToSpe.say("I have detect")
                # for i in range(len(drinks)):
                #     detect_drink.append(drinks[i][0])
                #     self.TextToSpe.say(drinks[i][0])
                #     print "yolo_detect_drink_name:",drinks[i][0]
                #if len(drinks)==0:
                    #self.TextToSpe.say("nothing")
                self.TextToSpe.say("on the table bar")
                for i in range(0,self.order_num):
                    print "in detect img guest_need:"+self.current_drink_name[i]
                    for index in range (len(detect_drink)):
                        if self.current_drink_name[i]==detect_drink[index]:
                            print "bar_detect order",self.current_drink_name[i]
                            self.order_in_bar.append(self.current_drink_name[i])
                        else:
                            print "bar_missing order",self.current_drink_name[i]
                            self.missing_order.append(self.current_drink_name[i])

        if len(self.order_in_bar)>0:
            self.hand_switch=True
            self.hand_over()
            self.TextToSpe.say("Dear opeartor please hand over the ")
            for order in  range(0,len(self.order_in_bar)):                       
                self.TextToSpe.say(" "+self.order_in_bar[order])
            self.TextToSpe.say("in my hand")
        if len(self.missing_order)>0:
            self.TextToSpe.say("The guest also need")
            for miss in range(0,len(self.missing_order)):
                self.TextToSpe.say(" "+self.missing_order[miss])
            self.TextToSpe.say("but there is none")

    def save_point_inverse(self, point):

        self.if_save_switch = True
        print "save_point"
        # amcl定位
        amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback_inverse)
        while self.if_save_switch:
            time.sleep(1)
        amcl_sub.unregister()

    def hand_over(self):
        arg = tuple([1])
        thread.start_new_thread(self.stretch_out_hand, arg)

    def stretch_out_hand(self, arg):

        print"set hand"
        names_R = ["RElbowRoll", "RElbowYaw", "RHand", "RShoulderPitch", "RShoulderRoll", "RWristYaw"]
        #names_L=["LElbowRoll", "LElbowYaw", "LHand", "LShoulderPitch", "LShoulderRoll", "LWristYaw"]
        angleLists_R = [89.3 * almath.TO_RAD, 117.6 * almath.TO_RAD, 0.39 * almath.TO_RAD, 24.3 * almath.TO_RAD, -84.3 * almath.TO_RAD, 77.7 * almath.TO_RAD]
        #angleLists_L = [-68.6 * almath.TO_RAD, -7.2 * almath.TO_RAD, 0.55 * almath.TO_RAD, 17.6 * almath.TO_RAD, 14.0 * almath.TO_RAD, -81.4 * almath.TO_RAD]
        timeLists = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        isAbsolute = True
        while self.hand_switch:
            self.Motion.angleInterpolation(names_R, angleLists_R, timeLists, isAbsolute)
            time.sleep(.1)
        #self.Motion.angleInterpolation(names_L, angleLists_L, timeLists, isAbsolute)

    def save_point(self):
        self.if_save_switch = True
        print "save_point"
        # amcl定位
        amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        while self.if_save_switch:
            time.sleep(1)
        amcl_sub.unregister()

    def head_fix_thread(self, arg):
        self.Motion.setStiffnesses("head", 1.0)
        while True:
            if self.head_fix:
                #print "=====self.angle:====", self.angle
                self.Motion.setAngles("Head", [0., self.angle], .2)
            time.sleep(3)

    def start_head_fix(self):
        arg = tuple([1])
        thread.start_new_thread(self.head_fix_thread, arg)

    def record_audio(self, hints, withBeep = True):
        # 亮灯
        self.Leds.on("MyGroup")
        if withBeep:
            # 参数 playSine(const int& frequence, const int& gain, const int& pan, const float& duration)
            self.AudioPla.playSine(1000, self.beep_volume, 1, .3)
        print('\033[0;32m [Kamerider I] start recording... \033[0m')
        channels = [0, 0, 1, 0]
        self.AudioRec.startMicrophonesRecording("/home/nao/audio/recog.wav", "wav", 16000, channels)
        while time.time() < self.record_time:
            if self.audio_terminate:
                # 如果终止为True
                self.AudioRec.stopMicrophonesRecording()
                print('\033[0;32m [Kamerider I] Killing recording... \033[0m')
                return None
            time.sleep(.1)
        self.Leds.off("MyGroup")
        self.AudioRec.stopMicrophonesRecording()
        self.AudioRec.recording_ended = True
        if not os.path.exists('./audio_record'):
            os.mkdir('./audio_record', 0o755)
        # 复制录下的音频到自己的电脑上
        cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":/home/nao/audio/recog.wav ./audio_record"
        os.system(cmd)
        print('\033[0;32m [Kamerider I] Record ended start recognizing \033[0m')
        self.recog_result = baidu_recognition_text.main("./audio_record/recog.wav").lower()
        print "===============", self.recog_result

    def analyze_content(self):
        result = []
        # 记录是否找到物品
        object_found = False
        # 获取要找的物品
        for i in range(len(self.object)):
            if re.search(self.object[i].lower(), self.recog_result) != None:
                print "found one person:=", self.object[i].lower()
                self.order_num+=1
                object_found = True
                # 记下当前物品的名字
                result.append(self.object[i])
        self.recog_result = "00"
        self.if_need_record=False
        if object_found:
            self.type = "FOUND"
            self.current_drink_name = result
            self.TextToSpe.say("OK, I will take ")
            for i in range(len(result)):
                self.TextToSpe.say(result[i] + " ")
            self.TextToSpe.say("to you. Please wait for a moment")
            print "order num",self.order_num
            return
        else:
            self.TextToSpe.say("sorry, please tell me again")
            self.start_recording(reset=True)
            self.analyze_content()

    def scan_callback(self, msg):
        print "in scan callback"
        print msg
        self.scan_msg.append(msg)
        self.scan_msg_time += 1
        if self.scan_msg_time == 4:
            self.scan_sub.unregister()

    def amcl_callback(self, msg):
        # print "yyyyyyyyy"
        # print msg
        # curr_pos = PoseStamped()
        point_temp = MoveBaseGoal()
        point_temp.target_pose.header.frame_id = '/map'
        point_temp.target_pose.header.stamp = msg.header.stamp
        point_temp.target_pose.header.seq = msg.header.seq
        point_temp.target_pose.pose.position.x = msg.pose.pose.position.x
        point_temp.target_pose.pose.position.y = msg.pose.pose.position.y
        point_temp.target_pose.pose.position.z = msg.pose.pose.position.z
        point_temp.target_pose.pose.orientation.x = msg.pose.pose.orientation.x
        point_temp.target_pose.pose.orientation.y = msg.pose.pose.orientation.y
        point_temp.target_pose.pose.orientation.z = msg.pose.pose.orientation.z
        point_temp.target_pose.pose.orientation.w = msg.pose.pose.orientation.w
        self.point_dataset.append(point_temp)
        print('\033[0;32m [Kamerider I] Point saved successfully!! \033[0m')
        self.if_save_switch = False

    def amcl_callback_inverse(self, msg):
        # print "yyyyyyyyy"
        # print msg
        # curr_pos = PoseStamped()
        point_temp = MoveBaseGoal()
        qua = transformations.quaternion_inverse([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        point_temp.target_pose.header.frame_id = '/map'
        point_temp.target_pose.header.stamp = msg.header.stamp
        point_temp.target_pose.header.seq = msg.header.seq
        point_temp.target_pose.pose.position.x = msg.pose.pose.position.x
        point_temp.target_pose.pose.position.y = msg.pose.pose.position.y
        point_temp.target_pose.pose.position.z = msg.pose.pose.position.z
        point_temp.target_pose.pose.orientation.x = qua[0]
        point_temp.target_pose.pose.orientation.y = qua[1]
        point_temp.target_pose.pose.orientation.z = qua[2]
        point_temp.target_pose.pose.orientation.w = qua[3]
        self.point_dataset.append(point_temp)
        print('\033[0;32m [Kamerider I] Point saved successfully!! \033[0m')
        self.if_save_switch = False

    def kill_recording_thread(self):
        if self.thread_recording.is_alive():
            self.audio_terminate = True
            self.if_need_record = False

    def go_to_waypoint(self, Point):
        self.angle = .3
        self.nav_as.send_goal(Point)
        self.map_clear_srv()
        count_time = 0
        # 等于3的时候就是到达目的地了
        while self.nav_as.get_state() != 3:
            count_time += 1
            time.sleep(1)
            # 每隔4s清除一次local map
            if count_time == 3:
                self.map_clear_srv()
                count_time = 0

    def stop_motion(self):
        # self.cancel_plan()
        self.set_velocity(0, 0, 0)

    def set_velocity(self, x, y, theta, duration=-1.):  # m/sec, rad/sec
        # if duration > 0 : stop after duration(sec)
        tt = Twist()
        tt.linear.x = x
        tt.linear.y = y
        tt.angular.z = theta
        self.cmd_vel_pub.publish(tt)
        if duration < 0: return None
        tic = time.time()
        while time.time() - tic < duration:
            self.cmd_vel_pub.publish(tt)
            time.sleep(0.1)
        tt = Twist()
        tt.linear.x = 0
        tt.linear.y = 0
        tt.angular.z = 0
        self.cmd_vel_pub.publish(tt)

    def keyboard_control(self):
        print('\033[0;32m [Kamerider I] Start keyboard control \033[0m')
        command = ''
        while command != 'c':
            try:
                command = raw_input('next command : ')
                if command == 'st':
                    self.start()
                elif command == 'w':
                    self.set_velocity(0.25, 0, 0)
                elif command == 's':
                    self.stop_motion()
                elif command == 'x':
                    self.set_velocity(-0.25, 0, 0)
                elif command == 'a':
                    self.set_velocity(0, 0.25, 0)
                elif command == 'd':
                    self.set_velocity(0, -0.25, 0)
                elif command == 'q':
                    self.set_velocity(0, 0, 0.35)
                elif command == 'e':
                    self.set_velocity(0, 0, -0.35)
                elif command == "save":
                    self.approach_waving()
                elif command == "print":
                    print self.point_dataset
                elif command == 'c':
                    break
                else:
                    print("Invalid Command!")
            except Exception as e:
                print e

        self.f.close()

if __name__ == "__main__":
    params = {
        'ip': "192.168.43.167",
        'port': 9559
    }
    res = restaurant(params)
    res.keyboard_control()



