#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
""""
代码名称：people_detection.py
代码功能：利用ALPeoplePerception的API接口实现Pepper机器人识别人体的功能
"""
import qi
import argparse
import sys
from colorama import Fore
import time


class people_detection:
    def __init__(self, session, ip, port):
        # self.if_in_callback变量确保程序一次只能进行一次回调函数，防止程序超线程
        self.if_in_callback = False
        self.init_param(session, ip, port)
        self.get_service()
        self.disable_auto_mode()
        self.set_parameter()
        self.start_people_detection()

    def init_param(self, session, ip, port):
        # 将主函数中的参数传入类内，以供后续使用
        self.ip = ip
        self.port = port
        self.session = session

    def get_service(self):
        # 获取本次案例中需要的服务
        self.RobotPos = self.session.service("ALRobotPosture")
        self.PeoplePer = self.session.service("ALPeoplePerception")
        self.AutonomousLife = self.session.service("ALAutonomousLife")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.Memory = self.session.service("ALMemory")

    def disable_auto_mode(self):
        # 取消机器人的自主模式，让机器人不会随着人转头，取消自主模式后重新站好
        print(Fore.GREEN + "[I]: 取消自主模式中")
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("StandInit", 0.5)

    def start_people_detection(self):
        # 开始人体检测
        self.PeoplePer.subscribe("People")
        self.People_Dete = self.Memory.subscriber("PeoplePerception/PeopleDetected")
        self.People_Dete.signal.connect(self.callback_people_dete)
        time.sleep(20)
        self.PeoplePer.unsubscribe("People")

    def set_parameter(self):
        # 设置成true时，所有其他可选的检测（比如脸、运动等）都将停用
        self.PeoplePer.setFastModeEnabled(True)

    def callback_people_dete(self, msg):
        if self.if_in_callback:
            return
        self.if_in_callback = True
        print (Fore.GREEN + "[I]: 检测到人")
        self.TextToSpe.say("people detected")
        time.sleep(2)
        self.if_in_callback = False  
         

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
    people_detector = people_detection(session, args.ip, args.port)
