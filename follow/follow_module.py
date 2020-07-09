#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
from time import sleep
from threading import Thread
import time


class pepper_follow:

    def __init__(self, session):
        self.follow_enable = False
        self.Motion = session.service("ALMotion")
        self.Tracker = session.service("ALTracker")
        self.RobotPos = session.service("ALRobotPosture")
        self.PeoplePer = session.service("ALPeoplePerception")
        self.TextToSpe = session.service("ALTextToSpeech")
        self.PeopPer = session.service("ALPeoplePerception")
        self.Memory = session.service("ALMemory")
        self.Time_ = time.time()
        self.PeoplePer.subscribe("Peoplee"+str(self.Time_))
        self.People_Dete = self.Memory.subscriber("PeoplePerception/PeopleDetected")
        self.People_Dete.signal.connect(self.callback_people_dete)
        # 设置成true时，所有其他可选的检测（比如脸、运动等）都将停用
        self.PeoplePer.setFastModeEnabled(True)
        # Set Stiffness
        name = "Body"
        stiffness = 1.0
        time_ = 1.0
        self.temp = 0
        self.Motion.stiffnessInterpolation(name, stiffness, time_)
        # Go to posture stand
        speed = 1.0
        self.people_id = 0
        # 设置追踪模式
        mode = "Move"
        self.Tracker.setMode(mode)

        print("                        ↓                            ")
        print('\033[0;32m [Kamerider I] follow function initialized \033[0m')

    def callback_people_dete(self, msg):
        print "00000000000000000000"
        self.people_id = msg[1][0][0]

    def follow(self, arg):
        print "111111================="
        # tracker_service.trackEvent("Face")
        while self.follow_enable:
            if self.people_id == 0:
                print("\033[0;32;40m\t[Kamerider W] : There is nobody in front of me\033[0m")
                time.sleep(2)
                # self.TextToSpe.say("I can't see you, please adjust the distance between us  ")
                continue
            else:
                self.Tracker.registerTarget("People", self.people_id)
                print "registe Target successfully!!"
                self.Tracker.track("People")
                self.TextToSpe.say("I am ready to follow you")
                self.TextToSpe.say("please lead me to the car position")
                self.TextToSpe.say("Please say stop when you want me stop")
                break
        while self.follow_enable:
            # print "111111"

            # 获得机器人躯干坐标系下距离目标的距离
            target_position = self.Tracker.getTargetPosition(0)
            if not target_position:
                continue
            # 距离大于1.7m
            if target_position[0] > 3:
                self.temp += 1
                if self.temp == 100:
                    print "+++++++++++++++++++++++++++++++BREAK+++++++++++++++++++++++++++++++++"
                    break
                print "-------------------------------------" + str(target_position[0])
                # self.TextToSpe.say("current distance is" + str(round(target_position[0], 1)))
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()

    def __del__(self):
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()
        self.PeoplePer.unsubscribe("Peoplee"+str(self.Time_))


