#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import os
import re
import cv2
import sys
import time
import rospy
import thread
import atexit
import actionlib
import numpy as np
from threading import Thread
from std_srvs.srv import Empty
from tf import transformations
from actionlib_msgs.msg import GoalID
from gender_predict import face_feature
from gender_predict import body_feature
from sensor_msgs.msg import LaserScan
from speech_recog import speech_recognition_text
# from speech_recog import baidu_recognition_text
from human_detection import human_detection
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Quaternion


class find_my_mate():

    def __init__(self, params):
        atexit.register(self.__del__)
        # pepper connection
        self.ip = params["ip"]
        self.port = params["port"]
        self.session = qi.Session()
        rospy.init_node("find_my_mate")
        try:
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print("[Kamerider E] : connection Error!!")
            sys.exit(1)
        self.car_pose = MoveBaseGoal()
        # naoqi API
        self.Leds = self.session.service("ALLeds")
        self.Memory = self.session.service("ALMemory")
        self.Dialog = self.session.service("ALDialog")
        self.Motion = self.session.service("ALMotion")
        self.AudioDev = self.session.service("ALAudioDevice")
        self.AudioPla = self.session.service("ALAudioPlayer")
        self.PhotoCap = self.session.service("ALPhotoCapture")
        self.VideoDev = self.session.service("ALVideoDevice")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.AudioRec = self.session.service("ALAudioRecorder")
        self.BasicAwa = self.session.service("ALBasicAwareness")
        self.TabletSer = self.session.service("ALTabletService")
        self.SoundDet = self.session.service("ALSoundDetection")
        self.AutonomousLife = self.session.service("ALAutonomousLife")
        # stop recording
        try:
            self.AudioRec.stopMicrophonesRecording()
        except BaseException:
            print("\033[0;33m\t[Kamerider W]ALFaceCharacteristics : You don't need stop record\033[0m")
        # 录音的函数
        self.thread_recording = Thread(target=self.record_audio, args=(None,))
        self.thread_recording.daemon = True
        self.audio_terminate = False
        # ROS 订阅器和发布器
        self.nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.nav_as.wait_for_server()
        self.if_door_open = False
        # 清除costmap
        self.map_clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.map_clear_srv()
        self.Motion.setTangentialSecurityDistance(.05)
        self.Motion.setOrthogonalSecurityDistance(.1)
        #    LED的group
        self.led_name = ["Face/Led/Green/Right/0Deg/Actuator/Value", "Face/Led/Green/Right/45Deg/Actuator/Value",
                         "Face/Led/Green/Right/90Deg/Actuator/Value", "Face/Led/Green/Right/135Deg/Actuator/Value",
                         "Face/Led/Green/Right/180Deg/Actuator/Value", "Face/Led/Green/Right/225Deg/Actuator/Value",
                         "Face/Led/Green/Right/270Deg/Actuator/Value", "Face/Led/Green/Right/315Deg/Actuator/Value",
                         "Face/Led/Green/Left/0Deg/Actuator/Value", "Face/Led/Green/Left/45Deg/Actuator/Value",
                         "Face/Led/Green/Left/90Deg/Actuator/Value", "Face/Led/Green/Left/135Deg/Actuator/Value",
                         "Face/Led/Green/Left/180Deg/Actuator/Value", "Face/Led/Green/Left/225Deg/Actuator/Value",
                         "Face/Led/Green/Left/270Deg/Actuator/Value", "Face/Led/Green/Left/315Deg/Actuator/Value"]
        self.Leds.createGroup("MyGroup", self.led_name)
        # 声明一些变量
        # amcl_pose话题的订阅器
        self.amcl_pose_sb = None
        self.position_result = []
        self.get_image_switch = True
        self.to_find_person_name = []
        self.current_drink_name = []
        # 保存当前对话人的年龄性别
        self.gender = "none"
        self.age = "none"
        # 存储每次语音识别的类别
        self.type = "none"
        # 储存每次语音识别的结果
        self.audio_recog_result = "none"
        # 最多接近多少次
        self.temp_reach_time = 0
        # 保存当前人的肤色，衣服颜色和类别
        self.lower_color = "none"
        self.lower_wear = "none"
        self.upper_color = "none"
        self.upper_wear = "none"
        self.skin_color  = "none"
        self.predefined_position = {
                # "bed":[0.752529621124,4.56971073151,0.0,0.0,0.875692237431,0.48286965664],
                # "shelf":[3.48698234558,5.21366643906,0.0,0.0,0.46922085657,0.88308085007],
                # "side table":[3.49552464485,4.00150775909,0.0,0.0,0.00539639848746,0.999985439336],
                # "sink":[4.11269855499,3.84168052673,0.0,0.0,0.86228371236,0.506425512192],
                # "dish washer":[4.36022758484,4.83986616135,0.0,0.0,-0.999998542378,0.00170740794843],
                # "fridge":[4.38763856888,5.69660758972,0.0,0.0,0.82926155938,0.558860685801],
                "trash bin":[4.34827899933,.21289503574,0.0,0.0,0.932419364911,0.361378095543],
                "tv":[6.94088697433,2.1938290596,0.0,0.0,0.387483707115,0.921876551779],
                "couch":[6.29112815857,-0.861555457115,0.0,0.0,-0.27312959294,0.961977247891],
                "arm chair":[7.84281349182,0.143020510674,0.0,0.0,-0.401161409027,0.916007382016]
                # "coat hanger":[1.34018230438,1.20021378994,0.0,0.0,0.905398107208,0.424563620042],
                # "desk":[2.42883086205,-0.940541505814,0.0,0.0,-0.704562317248,0.709642121857],
                # "kitchen table":[6.16149091721,3.65961480141,0.0,0.0,0.379552154941,0.92517034198],
                # "shoe rack":[0.840799331665,-0.702893853188,0.0,0.0,-0.95921175774,0.282688527912]
            }
        # 保存当前人旁边有什么东西
        self.position = "none"
        self.positive_list = ["yes", "of course", "we do"]
        self.negative_list = ["no", "sorry", "we don't", "regret"]
        self.name_list = ["Amelia","Angel","Charlie","Ava","Hunter","Jack","Charlotte","Max",
                          "Noah","Oliver","Mia", "Parker","Sam","Thomas","William", "check", "Olivia", "even"]
        self.angle = -.1
        self.person_num = 1
        self.head_fix = True
        self.if_need_record = False
        self.point_dataset = self.load_waypoint("waypoints_help.txt")
        # 人体识别
        self.human_detector = human_detection.human_detector()
        # 物体识别
        # self.object_detection = object_detection()
        # 关闭basic_awareness
        if self.BasicAwa.isEnabled():
            self.BasicAwa.setEnabled(False)
        if self.BasicAwa.isRunning():
            self.BasicAwa.pauseAwareness()
        # 关闭AutonomousLife模式-
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        # 初始化平板
        self.TabletSer.hideImage()
        print ('\033[0;32m [Kamerider I] Tablet initialize successfully \033[0m')
        # 设置dialog语言
        self.Dialog.setLanguage("English")
        # 录下的音频保存的路径
        self.audio_path = '/home/nao/audio/record.wav'
        self.recog_result = "00"
        self.RobotPos.goToPosture("StandInit", .2)
        # Beep 音量
        self.beep_volume = 70
        # 设置说话速度
        self.TextToSpe.setParameter("speed", 75.0)
        # 初始化录音
        self.record_delay = 2.5
        self.speech_hints = []
        self.enable_speech_recog = True
        # 1代表最灵敏
        self.SoundDet.setParameter("Sensitivity", .3)
        self.SoundDet.subscribe('sd')
        self.SoundDet_s = self.Memory.subscriber("SoundDetected")
        self.SoundDet_s.signal.connect(self.callback_sound_det)
        # 调用成员函数
        self.start_head_fix()
        self.set_volume(.7)
        self.show_image("instructions.png")
        self.keyboard_control()

    def __del__(self):
        print ('\033[0;32m [Kamerider I] System Shutting Down... \033[0m')
        self.AudioRec.stopMicrophonesRecording()
        cv2.destroyAllWindows()

    def start_head_fix(self):
        arg = tuple([1])
        thread.start_new_thread(self.head_fix_thread, arg)

    # def show_person_image(self, img_name):
    #     cmd = "sshpass -p kurakura326 scp ./person.jpg nao@" + str(self.ip) + \
    #           ":~/.local/share/PackageManager/apps/boot-config/html"
    #     os.system(cmd)
    #     self.TabletSer.hideImage()
    #     self.TabletSer.showImageNoCache("http://198.18.0.1/apps/boot-config/" + img_name)

    def show_image(self, image_name):
        # cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) +
        # ":./person_image/person_image.png ~/.local/share/PackageManager/apps/boot-config/html"
        # os.system(cmd)
        self.TabletSer.hideImage()
        time.sleep(1)
        self.TabletSer.showImageNoCache("http://198.18.0.1/apps/boot-config/" + str(image_name))

    def head_fix_thread(self, arg):
        self.Motion.setStiffnesses("head", 1.0)
        while True:
            if self.head_fix:
                # print "=====self.angle:====", self.angle
                self.Motion.setAngles("Head", [0., self.angle], .2)
            time.sleep(3)

    def say(self, text):
        self.TextToSpe.say(text)

    def load_waypoint(self, file_name):
        curr_pos = PoseStamped()
        f = open(file_name, 'r')
        sourceInLines = f.readlines()
        dataset_points = {}
        for line in sourceInLines:
            temp1 = line.strip('\n')
            temp2 = temp1.split(',')
            point_temp = MoveBaseGoal()
            point_temp.target_pose.header.frame_id = '/map'
            point_temp.target_pose.header.stamp = curr_pos.header.stamp
            point_temp.target_pose.header.seq = curr_pos.header.seq
            point_temp.target_pose.pose.position.x = float(temp2[1])
            point_temp.target_pose.pose.position.y = float(temp2[2])
            point_temp.target_pose.pose.position.z = float(temp2[3])
            point_temp.target_pose.pose.orientation.x = float(temp2[4])
            point_temp.target_pose.pose.orientation.y = float(temp2[5])
            point_temp.target_pose.pose.orientation.z = float(temp2[6])
            point_temp.target_pose.pose.orientation.w = float(temp2[7])
            dataset_points[temp2[0]] = point_temp
        print ("↓↓↓↓↓↓↓↓↓↓↓↓point↓↓↓↓↓↓↓↓↓↓↓↓")
        print (dataset_points)
        print ("↑↑↑↑↑↑↑↑↑↑↑↑point↑↑↑↑↑↑↑↑↑↑↑↑")
        print ('\033[0;32m [Kamerider I] Points Loaded! \033[0m')
        return dataset_points

    def set_volume(self, volume):
        self.TextToSpe.setVolume(volume)

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

    def cal_distence(self, position, orientation):
        goal = transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        min = 1000
        # 寻找距离最近的点
        for i in self.predefined_position.keys():
            dist = (self.predefined_position[i][0] - position[0])*(self.predefined_position[i][0] - position[0]) + \
                   (self.predefined_position[i][1] - position[1])*(self.predefined_position[i][1] - position[1])
            if dist < min:
                temp = transformations.euler_from_quaternion(
                    [self.predefined_position[i][2], self.predefined_position[i][3],
                     self.predefined_position[i][4], self.predefined_position[i][5]])
                if (temp[2] > 0 and goal[2] > 0) or (temp[2] < 0 and goal[2] < 0):
                    if abs(temp[2] - goal[2]) > 2.8 / 2:
                        continue
                    else:
                        min = dist
                        self.position = i
                elif (temp[2] > 0 and goal[2] < 0) or (temp[2] < 0 and goal[2] > 0):
                    if (6.28 - (abs(temp[2]) + abs(goal[2]))) < 2.8 / 2 or (abs(temp[2]) + abs(goal[2])) < 2.8 / 2:
                        min = dist
                        self.position = i
                    else:
                        continue
        # 防止返回sofa1和sofa2
        if self.position == "sofa1" or self.position == "sofa2":
            self.position = "sofa"

    def analyze_content(self):
        result = []
        # 记录是否找到人
        person_found = False
        # 获取要找的人姓名
        for i in range(len(self.name_list)):
            if re.search(self.name_list[i].lower(), self.recog_result) != None:
                print "found one person:=", self.name_list[i].lower()
                person_found = True
                # 记下当前人的名字
                if self.name_list[i] == "check":
                    self.name_list[i] = "jack"
                if self.name_list[i] == "even":
                    self.name_list[i] = "ava"
                result.append(self.name_list[i])

        self.recog_result = "00"
        if person_found:
            self.type = "NAME"
            self.audio_recog_result = result
            return
        else:
            self.say("sorry please tell me more quickly, please tell me again")
            self.start_recording(reset=True)
            self.analyze_content()

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
        self.recog_result = speech_recognition_text.main("./audio_record/recog.wav").lower()
        print "===============", self.recog_result

    def take_picture(self):
        self.PhotoCap.takePictures(3, '/home/nao/picture', 'party')
        cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":/home/nao/picture/party_0.jpg ./person_image"
        os.system(cmd)

        # cmd = "sshpass -p kurakura326 scp ./person_image/party_0.jpg nao@" + str(
        #     self.ip) + ":~/.local/share/PackageManager/apps/boot-config/html/"
        # os.system(cmd)
        # 把人的面部特征保存
        self.gender, self.age, self.skin_color = face_feature.gender("./person_image/party_0.jpg", self.upper_wear, self.upper_color, self.person_num)
        if self.gender == "none":
            self.say("please look at my eyes")
            self.take_picture()
        else:
            return self.gender

    def add_name_to_img(self, img_name, position, person_name, posture):
        img = cv2.imread(img_name)
        cv2.putText(img, "Name:"+str(person_name), (160, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        cv2.putText(img, "Position:"+posture+" next to the "+str(position), (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        cv2.imwrite(img_name, img)

    def amcl_pose_cb(self, msg):
        qua = Quaternion()
        position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        qua.x = msg.pose.pose.orientation.x
        qua.y = msg.pose.pose.orientation.y
        qua.z = msg.pose.pose.orientation.z
        qua.w = msg.pose.pose.orientation.w
        self.cal_distence(position, qua)
        self.amcl_pose_sb.unregister()

    def laser_callback(self, msg):
        # 检测中间的laser的信息判断门是不是开着的
        sum = 0
        num_laser = 0
        # 前面的laser的数据
        for i in range(23, 37):
            if msg.ranges[i] > 5:
                continue
            else:
                num_laser += 1
                sum += msg.ranges[i]
        print "current_sum_average=:",sum / num_laser
        # 大于阈值就认定门开了
        if sum / num_laser > 1:
            self.if_door_open = True
            self.laser_sb.unregister()

    def start(self):
        # 开门进屋
        # 订阅laser的话题，判断门是不是开着的
        self.TextToSpe.say("i am ready, please help me open the door.")
        self.laser_sb = rospy.Subscriber('/pepper_robot/naoqi_driver/laser', LaserScan, self.laser_callback)
        print "start"
        while not self.if_door_open:
            time.sleep(.5)
            # print "waiting the door"
        self.TextToSpe.say("the door is open")
        self.map_clear_srv()
        time.sleep(1)

        self.go_to_waypoint(self.point_dataset["point62"], "point2", "first")
        self.go_to_waypoint(self.point_dataset["point58"], "point2", "first")
        # 抬头
        self.angle = -.5
        self.save_point()
        self.Motion.setAngles("Head", [0., self.angle], .2)
        self.TextToSpe.say("Dear operator.")
        self.TextToSpe.say("Please talk to me after my eyes' color turn to white ")
        self.TextToSpe.say("now, Please give me three names you want to find")
        self.to_find_person_name = ["test"]
        while True:
            # 开始和operator对话
            self.start_recording(reset=True)
            self.analyze_content()
            self.to_find_person_name = self.audio_recog_result
            break
        self.say("ok, I will find ")
        for i in range(len(self.to_find_person_name)):
            self.say(str(self.to_find_person_name[i]) + " ")
        self.TextToSpe.say("excuse me, please let the way for me")
        self.map_clear_srv()
        time.sleep(2)
        # 走进屋子，开始找人
        self.go_to_waypoint(self.point_dataset["point34"], "point2", "first")
        self.go_to_waypoint(self.point_dataset["point68"], "point2", "first")
        person_found_num = 0
        while person_found_num != len(self.to_find_person_name):
            self.find_person()
            self.get_image_switch = True
            self.amcl_pose_sb = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_cb)
            # 如果找到的人是我们的list的人
            if self.dialog_with_people() == "succe":
                person_found_num += 1
                self.Motion.moveTo(0, 0, -3.14 / 4)
                self.show_image("instructions.png")
            else:
                self.Motion.moveTo(0, 0, -3.14 / 4)
                self.show_image("instructions.png")
                continue
        # 回到operator那里
        self.Motion.moveTo(0,0,3)
        self.go_to_waypoint(self.point_dataset["point33"], "point2", "first")
        self.go_to_waypoint(self.point_dataset["point58"], "point2", "first")
        self.angle = -.5
        for i in range(len(self.position_result)):
            self.show_image("person_result"+str(i+1)+".jpg")
            self.say(self.position_result[i])
        # 清空
        self.position_result = []

        # 完成了两次，开始找第三个人
        if len(self.to_find_person_name) == 1:
            self.TextToSpe.say("Now I am going to find the next person")
        else:
            self.TextToSpe.say("Dear operator, I do not remember the next person's name, could you tell me again?")
            self.start_recording(reset=True)
            self.analyze_content()
            self.to_find_person_name = self.audio_recog_result
        self.say("I will find ")
        for i in range(len(self.to_find_person_name)):
            self.say(str(self.to_find_person_name[i]) + " ")
        # 走进屋子，开始找人
        self.go_to_waypoint(self.point_dataset["point34"], "point2", "first")
        self.go_to_waypoint(self.point_dataset["point22"], "point2", "first")
        person_found_num = 0
        while person_found_num != 1:
            self.find_person()
            self.get_image_switch = True
            self.amcl_pose_sb = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_cb)
            self.show_image("instructions.png")
            # 如果找到的人是我们的list的人
            if self.dialog_with_people() == "succe":
                person_found_num += 1
                # 将当前人的信息传输到Pepper上
                cmd = "sshpass -p kurakura326 scp ./person_result3.jpg nao@" + str(
                    self.ip) + ":~/.local/share/PackageManager/apps/boot-config/html/"
                os.system(cmd)
            else:
                self.Motion.moveTo(0, 0, -3.14 / 4)
                continue
        # 回到operator那里
        self.Motion.moveTo(0,0,3)
        self.go_to_waypoint(self.point_dataset["point33"], "point2", "first")
        self.go_to_waypoint(self.point_dataset["point56"], "point2", "first")
        self.go_to_waypoint(self.point_dataset["point52"], "point2", "first")
        self.go_to_waypoint(self.point_dataset["point48"], "point2", "first")
        self.go_to_waypoint(self.point_dataset["start"], "point2", "first")
        self.angle = -.5
        for i in range(len(self.position_result)):
            self.show_image("person_result3.jpg")
            self.say(self.position_result[i])

    def save_point(self):
        self.if_save_switch = True
        print "save_point"
        # amcl定位
        amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        while self.if_save_switch:
            time.sleep(1)
        amcl_sub.unregister()

    def amcl_callback(self, msg):
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
        self.point_dataset["start"] = point_temp
        print('\033[0;32m [Kamerider I] Point saved successfully!! \033[0m')
        self.if_save_switch = False

    def stop_motion(self):
        self.goal_cancel_pub.publish(GoalID())
        self.set_velocity(0, 0, 0)

    def dialog_with_people(self):
        # 抬头
        # self.angle = -.5
        # self.Motion.setAngles("Head", [0., self.angle], .1)
        # time.sleep(1)
        # 拍照识别性别
        self.take_picture()
        # self.show_image("party_0.jpg")
        self.say("Hi, my name is pepper, what is your name?")
        self.start_recording(reset=True)
        self.analyze_content()
        if self.type == "NAME":
            for i in range(len(self.to_find_person_name)):
                # 如果这个人在要找的list里面
                print "========================match========================"
                if re.search(self.audio_recog_result[0].lower(), self.to_find_person_name[i].lower()) != None:
                    self.say("ok, I have remembered your position")
                    if self.angle < -0.25:
                        posture = "standing"
                    else:
                        posture = "sitting"
                    # 添加名字和位置，然后储存
                    self.add_name_to_img("person_result" + str(self.person_num) + ".jpg", self.position, self.to_find_person_name[i].lower(), posture)
                    # 把图片传进去
                    cmd = "sshpass -p kurakura326 scp ./person_result"+str(self.person_num)+".jpg nao@" + str(self.ip) + \
                          ":~/.local/share/PackageManager/apps/boot-config/html"
                    os.system(cmd)
                    # num数加1
                    self.person_num += 1
                    if self.gender == "male":
                        sentence = "the person named " + self.to_find_person_name[i].lower() + " is " + self.gender + ". And he is wearing " + \
                                   self.upper_color + " " + self.upper_wear + ". And he is "+ posture +" next to " + self.position
                    else:
                        sentence = "the person named " + self.to_find_person_name[i].lower() + " is " + self.gender + ". And she is wearing " + \
                                   self.upper_color + " " + self.upper_wear + ". And she is "+ posture +" next to " + self.position
                    print "---------------------------------------------------result---------------------------------------------------"
                    print sentence
                    self.position_result.append(sentence)
                    # 清空上一个人的数据
                    self.to_find_person_name.remove(self.to_find_person_name[i])
                    print "self.to_find_person_name", self.to_find_person_name
                    self.upper_color = self.upper_wear = self.lower_wear = self.lower_color = self.position = self.gender = "none"
                    # 找到了我们要找的人，返回“succe”
                    return "succe"
            self.TextToSpe.say("ok, thank you")
            # 最终没有找到我们要的人，返回“wrong”
            return "wrong"

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

    def kill_recording_thread(self):
        if self.thread_recording.is_alive():
            self.audio_terminate = True
            self.if_need_record = False

    def split_person_from_img(self, img, img_name, rect):
        new_img = img.copy()
        new_img = new_img[int(rect[1] - 10):int(rect[3] + 10), int(rect[0] - 10):int(rect[2] + 10)]
        cv2.imwrite(img_name, new_img)

    def find_person(self):
        self.angle = 0
        self.Motion.setAngles("Head", [0., self.angle], .2)
        AL_kQVGA = 2
        current_right = current_left = current_bottom = current_top = 0
        # Need to add All color space variables
        AL_kRGBColorSpace = 13
        fps = 60
        nameId = self.VideoDev.subscribe("image" + str(time.time()), AL_kQVGA, AL_kRGBColorSpace, fps)
        # create image
        width = 640
        height = 480
        image = np.zeros((height, width, 3), np.uint8)
        if_turn_finished = False
        if_first = True
        while self.get_image_switch:
            print "---------------------------------------self.angle", self.angle
            result = self.VideoDev.getImageRemote(nameId)
            if result == None:
                print 'cannot capture.'
            elif result[6] == None:
                print 'no image data string.'
            else:
                values = map(ord, list(str(bytearray(result[6]))))
                i = 0
                for y in range(0, height):
                    for x in range(0, width):
                        image.itemset((y, x, 0), values[i + 0])
                        image.itemset((y, x, 1), values[i + 1])
                        image.itemset((y, x, 2), values[i + 2])
                        i += 3
                # print image
                cv2.imshow("pepper-top-camera-640*480px", image)
                cv2.imwrite("./human_detection.jpg", image)
                cv2.waitKey(1)
                # dlib检测人脸
                rects = self.human_detector.main("./human_detection.jpg")
                print rects
                # 检测到人脸
                if len(rects) != 0:
                    if_first = True
                    # 转向完成，开始接近人
                    if if_turn_finished:
                        # 第一次看到人，就进行一次特征识别
                        if (self.upper_wear == "none" and self.upper_color == "none"):

                            image_name = "/home/fansa/Src/pepper_example/RoboCup2019/person_body.jpg"
                            # cv2.imwrite(image_name, image)
                            # self.position = self.object_detection.main(image_name)
                            self.split_person_from_img(image, image_name, rects)
                            _, _, self.upper_color, self.upper_wear = body_feature.feature(image_name)
                            if self.upper_color == "none" and self.upper_wear == "none":
                                # self.Motion.moveTo(-.5, 0, 0)
                                self.angle -= .05
                                self.Motion.setAngles("Head", [0., self.angle], .2)
                                continue
                        # 人的宽度所占的比例
                        print "rate=====================", float(rects[2] - rects[0]) / float(width)
                        if self.temp_reach_time == 15:
                            self.temp_reach_time = 0
                            self.set_velocity(0, 0, 0)
                            self.get_image_switch = False
                        if float(rects[2] - rects[0]) / float(width) > .3:
                            self.temp_reach_time = 0
                            self.set_velocity(0, 0, 0)
                            self.get_image_switch = False
                        else:
                            self.temp_reach_time += 1
                            # 通过人体框的上边框判断是否需要抬头
                            if rects[1] < height * 0.204:
                                print "upupupupupupupupupupupupupupup", current_bottom
                                self.angle -= .05
                                self.Motion.setAngles("Head", [0., self.angle], .2)
                            elif rects[1] > height * 0.4125:
                                print "downdowndowndowndowndowndowndown", current_top
                                self.angle += .05
                                self.Motion.setAngles("Head", [0., self.angle], .2)
                            self.set_velocity(.15, 0, 0)
                            # 再旋转
                            center = (rects[0] + rects[2]) / 2
                            if abs(width / 2 - center) > width / 10:
                                print "inininininininininininnnininininini"
                                Error_dist_ = width / 2 - center
                                self.set_velocity(0, 0, 0.001*Error_dist_)
                                time.sleep(1.8)
                                self.set_velocity(0.1, 0, 0)
                        cv2.waitKey(1)
                    # 开始转向人
                    else:
                        # 通过人体框的上边框判断是否需要抬头
                        if rects[1] < height * 0.204:
                            print "upupupupupupupupupupupupupupup", current_bottom
                            self.angle -= .05
                            self.Motion.setAngles("Head", [0., self.angle], .2)
                        elif rects[1] > height * 0.4125:
                            print "downdowndowndowndowndowndowndown", current_top
                            self.angle += .05
                            self.Motion.setAngles("Head", [0., self.angle], .2)

                        human_center = (rects[2] + rects[0]) / 2
                        Error_dist = width / 2 - human_center
                        if abs(Error_dist) <= 10:
                            if_turn_finished = True
                            continue
                        self.Motion.moveTo(0, 0, 0.002*Error_dist)
                        cv2.waitKey(1)
                # 没有检测到人脸就旋转
                else:
                    if if_first:
                        if_first = False
                        continue
                    self.Motion.moveTo(0, 0, -3.14 / 4)
        return "succe"

    def go_to_waypoint(self, Point, destination, label="none"): # Point代表目标点 destination代表目标点的文本 label
        # 设置头的角度
        self.angle = .3
        self.Motion.setAngles("Head", [0., self.angle], .2)
        # 发送目标点
        self.nav_as.send_goal(Point)
        # 清空局部地图
        self.map_clear_srv()
        count_time = 0
        # 等于3的时候就是到达目的地了
        while self.nav_as.get_state() != 3:
            count_time += 1
            time.sleep(1)
            if count_time == 8:
                self.map_clear_srv()
                count_time = 0
        # self.TextToSpe.say("I have arrived at " + destination)
        if label == "none":
            return

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
                elif command == 'get':
                    self.Motion.moveTo(1,-1,0)
                elif command == 'c':
                    break
                elif command == 'fp':
                    self.Motion.moveTo(0,0,3.14/4)
                elif command == 'sp':
                    self.start_recording(reset=True)
                else:
                    print("Invalid Command!")
            except Exception as e:
                print e


if __name__ == "__main__":
    params = {
        'ip': "10.10.62.4",
        'port': 9559
    }
    find_my_mate(params)