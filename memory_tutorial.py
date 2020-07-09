#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-
'''
代码名称：memory_tutorial.py
代码功能：学习机器人消息机制
'''
import qi
import atexit
import time
import sys
import argparse
from colorama import Fore


class memory:
    def __init__(self, session, ip, port):
        atexit.register(self.__del__)
        self.count = 0
        self.init_param(session, ip, port)
        self.get_service()
        self.disable_auto_mode()
        self.set_parameter()
        self.data_process()

    def get_service(self):
        # 获取本次案例中需要的服务
        self.Memory = self.session.service("ALMemory")
        self.TexToSpe = self.session.service("ALTextToSpeech")
        self.AutonomousLife = self.session.service("ALAutonomousLife")
        self.RobotPos = self.session.service("ALRobotPosture")

    def disable_auto_mode(self):
        # 取消机器人的自主模式，让机器人不会随着人转头，取消自主模式后重新站好
        print(Fore.GREEN + "[I]: 取消自主模式中")
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("StandInit", 0.5)

    def init_param(self, session, ip, port):
        # 将主函数中的参数传入类内，以供后续使用
        self.ip = ip
        self.port = port
        self.session = session

    def set_parameter(self):
        # 设置机器人说话的语言和语速
        self.TexToSpe.setParameter("speed", 75.0)
        self.TexToSpe.setLanguage("English")

    def data_process(self):
        # 利用getData函数可以直接获得消息的内容
        isButtonPressed = self.Memory.getData("ChestButtonPressed")
        print(Fore.GREEN + "[I]: 通过getData获得的数据是： " + str(isButtonPressed))
        # 若绑定回调函数，只有在消息触发时会激活回调函数，并把消息一并传送至回调函数中
        buttonPressed_sub = self.Memory.subscriber("ChestButtonPressed")
        buttonPressed_sub.signal.connect(self.callback_buttonPressed)
        while self.count <= 6:
            self.count += 1
            time.sleep(5)
        
    def callback_buttonPressed(self, msg):
        if msg != 0.0:
            print(Fore.GREEN + "[I]: 通过回调函数获取的数据是： " + str(msg))

    def __del__(self):
        print(Fore.GREEN + "[I]: 程序运行结束")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="机器人的IP地址用于连接Pepper，运行在机器人身上时使用默认 IP：127.0.0.1")
    parser.add_argument("--port", type=int, default=9559,
                        help="端口号码(PORT)")
    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print(Fore.RED + "[E]: 连接失败" )
        sys.exit(1)
    move_intance = memory(session, args.ip, args.port)