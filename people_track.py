#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
""""
代码名称：people_track.py
代码功能：利用ALTracker的API接口实现Pepper机器人追踪人体的功能
"""
import qi
import argparse
import sys
from colorama import Fore
import thread
import atexit
import time


class pepper_follow:
    def __init__(self, session, ip, port):
        atexit.register(self.__del__)
        # self.follow_enable为Track功能的开关
        self.follow_enable = False
        # 注册Track目标时使用
        self.people_id = -1
        self.init_param(session, ip, port)
        self.get_service()
        self.disable_auto_mode()
        self.start_people_detection()
        self.set_parameter()
        self.start_track()

    def init_param(self, session, ip, port):
        # 将主函数中的参数传入类内，以供后续使用
        self.ip = ip
        self.port = port
        self.session = session

    def get_service(self):
        # 获取本次案例中需要的服务
        self.Tracker = self.session.service("ALTracker")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.PeoplePer = self.session.service("ALPeoplePerception")
        self.AutonomousLife = self.session.service("ALAutonomousLife")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.Memory = self.session.service("ALMemory")
        self.Motion = self.session.service("ALMotion")

    def disable_auto_mode(self):
        # 取消机器人的自主模式，让机器人不会随着人转头，取消自主模式后重新站好
        print(Fore.GREEN + "[I]: 取消自主模式中")
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("StandInit", 0.5)

    def start_people_detection(self):
        # Pepper追踪人之前需要先注册目标，因此我们需要先进行人体检测，获取到目标人体的id来进行目标的注册
        self.PeoplePer.subscribe("People")
        self.People_Dete = self.Memory.subscriber("PeoplePerception/PeopleDetected")
        self.People_Dete.signal.connect(self.callback_people_dete)

    def set_parameter(self):
        # 设置成true时，所有其他可选的检测（比如脸、运动等）都将停用
        self.PeoplePer.setFastModeEnabled(True)
        # 设置追踪模式
        mode = "Move"
        self.Tracker.setMode(mode)

    def start_track(self):
        # 开始追踪功能
        self.follow_enable = True
        arg = tuple([1])
        thread.start_new_thread(self.follow, arg)
        # 进行30s
        time.sleep(30)
        # 停止追踪功能
        self.follow_enable = False

    def callback_people_dete(self, msg):
        print (Fore.GREEN + "[I]: 检测到人")
        self.people_id = msg[1][0][0]

    def follow(self, arg):
        # 开始Track功能之前，检测是否已经检测到人
        while self.follow_enable:
            if self.people_id == -1:
                print(Fore.YELLOW + "[W]: 没有检测到人，无法开始Track功能。请前后调整一下。")
                time.sleep(1)
                continue
            else:
                self.Motion.moveInit()
                self.Tracker.registerTarget("People", self.people_id)
                print(Fore.GREEN + "[I]: 目标注册成功，开始Track功能")
                break
        # 开始Track功能
        self.Tracker.track("People")
        while self.follow_enable:
            # 获得机器人躯干坐标系下距离目标的距离
            target_position = self.Tracker.getTargetPosition(0)
            if not target_position:
                continue
            # 距离大于2m，进行语音提醒
            if target_position[0] > 2:
                self.TextToSpe.say("please slow down, I can not follow you.")
        #self.Tracker.unsubscribe("people-track")
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()

    def __del__(self):
        print (Fore.GREEN + "[I]: 退出程序中……")
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()
        self.PeoplePer.unsubscribe("People")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                      help="机器人的 IP 地址用于连接 Pepper，运行在机器人上时使用默认 IP：127.0.0.1")
    parser.add_argument("--port", type=int, default=9559,
                        help="端口号码（PORT）")
    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print (Fore.RED + "在 IP 为：  \"" + args.ip + "\"  ，端口为  " + str(args.port) +"的情况下无法连接到 NAOqi\n请检查IP和PORT是否正确。")
        sys.exit(1)
    follow_model = pepper_follow(session, args.ip, args.port)
