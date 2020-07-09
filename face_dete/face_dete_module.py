#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import qi
import rospy
import time
from threading import Thread
from geometry_msgs.msg import Twist


class face_dete:
    def __init__(self, session):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.FaceDet = session.service("ALFaceDetection")
        self.Tracker = session.service("ALTracker")
        self.Memory = session.service("ALMemory")
        self.Motion = session.service("ALMotion")
        self.TextToSpe = session.service("ALTextToSpeech")
        # 人脸检测部分
        if self.FaceDet.isRecognitionEnabled():
            # 可以让检测速度更快
            self.FaceDet.setRecognitionEnabled(False)
        # face detection 回调函数
        self.FaceDet.subscribe("HumanGreeter")
        self.Face_Dete = self.Memory.subscriber("FaceDetected")
        self.Face_Dete.signal.connect(self.callback_face_dete)
        # 开关
        self.switch_face_dete = False
        self.Tracker.registerTarget("Face", .2)
        self.Tracker.setMode("Navigate")
        # self.Tracker.trackEvent("Face")
        self.face_id = 0

    def start_face_dete(self):
        num = 0
        ro_angle = -.25
        while self.switch_face_dete:
            print "------------------"
            if self.face_id == 0:
                num += 1
                if num == 6:
                    ro_angle = -ro_angle
                    self.set_velocity(0, 0, ro_angle, 5)
                    continue
                self.set_velocity(0, 0, ro_angle, 1)
                time.sleep(1)
            else:
                break
        self.Tracker.trackEvent("Face")
        self.TextToSpe.say("Hey! I'm going to your position")

        # 小于阈值的次数
        num = 0
        while self.switch_face_dete:
            target_position = self.Tracker.getTargetPosition(0)
            print target_position
            if not target_position:
                continue
            if target_position[0] < 1.3 and target_position[2] > 0.9:
                num += 1
                if num > 4:
                    self.Tracker.stopTracker()
                    self.Tracker.unregisterAllTargets()
                    self.TextToSpe.say("Hey! I have reached your position")
                    self.TextToSpe.say("please follow me to the car position to help me carry something")
                    break
            else:
                self.Motion.moveTo(1, 0, 0)
                time.sleep(1)

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

    def callback_face_dete(self, msg):
        try:
            faceInfoArray = msg[1]
            for j in range( len(faceInfoArray)-1 ):
                print len(faceInfoArray)-1
                faceInfo = faceInfoArray[j]
                faceExtraInfo = faceInfo[1]
                print "face num", len(faceInfoArray) - 1
                self.face_id = faceExtraInfo[0]
        except IndexError:
            print "IndexError"


