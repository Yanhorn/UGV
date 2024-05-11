#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@File    : IimtArm3.py
@Time    : 2023/02/20 09:37:28
@Author  : Ding ruichen
@Contact : 2485525984@qq.com
@Version : 0.3
@Language: python3.8
@Desc    : 
..      三代机械臂(通信确定版)socket功能封装
..      2023.2.21
..          问    题 : 解决了线程开的时候,移动机械臂或者拔网线,getData()传输的信息都不会变的问题
..          解决方法  : getData()中的time.sleep()时间应该调的和机械臂发送数据时间一样
..
..      2023.3.14
..          修改内容 : 
..                  1. getData()获取机械臂信息时不需要在里面加上 while True:
..                  2. 各模块调用测试 
"""

import time
from tools.mysocket import socketTools
from struct import Struct
import numpy as np
import math
import threading

from tools.utils import clearSocketBuffer

# 上一个递归锁,虽然没什么必要
coboot_lock = threading.RLock()

class iimtCobot(socketTools):
    def __init__(self, ip: str = "192.168.1.40",
                 port: int     = 30003,
                 thread_time: float = 0.09,
                 block_com: bool    = False):
        super().__init__(ip, port, thread_time, block_com)

        # 机械臂线程回调数据
        self.__arm_callback_data = None
        self.__block_com = block_com
        # 是否清空缓存模式
        self.clear_socket = True
        
    #------------------------------------------------#
    #              获取机械臂socket属性                #
    #------------------------------------------------#    
    def update_cb_data(self, data):
        with coboot_lock:
            self.__arm_callback_data = data
            
    def get_cb_data(self):
        with coboot_lock:
            return self.__arm_callback_data
    #------------------------------------------------#
    #                机械臂的方法                      #
    #------------------------------------------------#
    def setStart(self):
        '''机械臂启动路点程序'''
        try:
            mess = "start()"
            # 定义socket的通信句柄 在/tools/mysocket.py中定义的socket句柄
            self.client_socket_.send(mess.encode())
            return True, None, f"[{mess}] 机械臂启动路点程序成功"
        except Exception as e:
            return False,None,f"[{mess}] 机械臂启动路点程序失败 {Exception}_{e}"

    def setStopJ(self):
        '''机械臂停止路点程序和停止机械臂动作'''
        try:
            mess = "stopj()"
            self.client_socket_.send(mess.encode())
            if self.is_alive():
                out = self.waitArmStart(expect_state=4)
                if out:
                    return True, None, f"[{mess}] 机械臂stopj成功状态为 4!"
                else:
                    return False, None, f"[{mess}] 机械臂stopj失败!"
                
            return True, None, f"[{mess}] 机械臂路点程序停止成功!"
        except Exception as e:
            return False, None, f"[{mess}t] 机械臂路点程序停止失败 {Exception}_{e}"

    def setPowerOn(self):
        '''机械臂上电'''
        try:
            mess = "robotPowerOn()"
            self.client_socket_.send(mess.encode())
            return True, None, f"[{mess}] 机械臂上电成功!"
        except Exception as e:
            return False, None, f"[{mess}] 机械臂上电失败{Exception}_{e}!"

    def setFastPowerOn(self):
        """机械臂上电,返回一个三元字符"""
        try:
            # 首先需要上电信号
            mess = "robotPowerOn()\n"
            self.client_socket_.send(mess.encode())
            
            # 等待心跳信号回来
            time.sleep(5)
            mess = "robotReleaseBrake()\n"
            self.client_socket_.send(mess.encode())

            # 执行这个指令后才能走movel和movej 
            #------修改时间 2023/02/21 19:22:08 夏炎 wechat:18261953682 --------
            #------修改内容 setAutoRun经过协作修改已经不需要就能启动机械臂了
            # time.sleep(1)
            # mess = "setAutoRun()\n"
            # self.client_socket_.send(mess.encode())
            if self.is_alive():
                out = self.waitArmStart(expect_state=4)
                if out:
                    return True, None, f"[{mess}] 机械臂上电成功!"
                else:
                    return False, None, f"[{mess}] 机械臂上电失败!"
            return True, None, "robotPowerOn()&robotReleaseBrake()&setAutoRun()指令发送完成\n"
        except Exception as e:
            return False, None, f"[{mess}] robotPowerOn()&robotReleaseBrake()&setAutoRun()指令发送失败{Exception}_{e}!"
        
    def setPowerOff(self):
        '''机械臂关电 如果你开线程那么可以用状态判断'''
        try:
            mess = "robotPowerOff()"
            self.client_socket_.send(mess.encode())
            
            if self.is_alive():
                out = self.waitArmStart(expect_state=0)
                if out:
                    return True, None, f"[{mess}] 机械臂关电成功!"
                else:
                    return False, None, f"[{mess}] 机械臂关电失败!"
            return True, None, f"[{mess}] 机械臂关电成功!"
        except Exception as e:
            return False, None, f"[{mess}] 机械臂关电失败{Exception}_{e}!"

    def setStop(self):
        '''机械臂掉电'''
        try:
            mess = "stop()"
            self.client_socket_.send(mess.encode())
            return True, None, f"[{mess}] 机械臂急停成功!"
        except Exception as e:
            return False, None, f"[{mess}] 机械臂急停失败{Exception}_{e}!"

    def setReleaseBrake(self):
        '''机械臂释放抱闸'''
        try:
            mess = "robotReleaseBrake()"
            self.client_socket_.send(mess.encode())
            return True, None, f"[{mess}] 机械臂释放抱闸成功!"
        except Exception as e:
            return False, None, f"[{mess}] 机械臂释放抱闸失败{Exception}_{e}!"

    def setFreeDriveMode(self):
        """机械臂自由拖动模式"""
        try:
            mess = "robotFreeDriveMode()"
            self.client_socket_.send(mess.encode())
            return True, None, f"[{mess}] 启动自由拖动模式!"
        except Exception as e:
            return False, None, f"[{mess}] 启动自由拖动模式失败{Exception}_{e}!"

    def setendFreeDrivMode(self):
        """机械臂结束自由拖动"""
        try:
            mess = "endFreeDriveMode()"
            self.client_socket_.send(mess.encode())
            return True, None, f"[{mess}] 结束自由拖动模式!"
        except Exception as e:
            return False, None, f"[{mess}] 结束自由拖动模式失败{Exception}_{e}!"

    def setAutoRun(self):
        """设置为自动运行模式,发送节点坐标之前的必须操作"""
        try:
            mess = "setAutoRun()"
            self.client_socket_.send(mess.encode())
            self.AutoRunMode_flag = True
            return True, None, f"[{mess}] 启动自动运行模式成功!"
        except Exception as e:
            return False, None, f"[{mess}] 启动自动运行模式失败{Exception}_{e}!"

    def setmovel(self, toolpose, num1='15', num2='100', num3='100', num4='0', num5='0'):
        '''
        @description:
            让机械臂以末端移动
        @Args:
            toolpose: 机械臂末端位姿数组,x,y,z单位是米,rx,ry,rz单位是示教器上的值,包含x,y,z,rx,ry,rz
            num1:     速度
            num2:     加速度
            num3:     加加速度
        @Return:
            标志位,None,状态说明反馈
        '''
        try:
            x,y,z,rx,ry,rz=toolpose
            mess = "movel({"+f"{x},{y},{z},{rx},{ry},{rz}"+"},"+f"{num1},{num2},{num3},{num4},{num5})"
            self.client_socket_.send(mess.encode())
            if self.is_alive():
                out = self.pose_guard(toolpose, movej = False)
                if out:
                    return True, None, f"[{mess}] 机械臂达位成功!"
                else:
                    return False, None, f"[{mess}] 机械臂达位失败{Exception}_{e}!"
                
            return True,None,f"{mess}发送成功!"
        except Exception as e:
            return False,None,f"[{mess}] movel发送失败 {Exception}_{e}"
        
    def setmovej(self, joint, num1='15', num2='100', num3='100', num4='0', num5='0'):
        '''
        @description:
            让机械臂以关节移动
        @Args:
            joint: 机械臂六个关节的弧度数组,单位为弧度,包含joint1,joint2,joint3,joint4,joint5,joint6
            num1:  速度
            num2:  加速度
            num3:  加加速度
        @Return:
            标志位,None,状态说明反馈
        '''
        try:
            joint1,joint2,joint3,joint4,joint5,joint6 = joint
            mess = "movej({"+f"{joint1},{joint2},{joint3},{joint4},{joint5},{joint6}"+"},"+f"{num1},{num2},{num3},{num4},{num5})"
            self.client_socket_.send(mess.encode())
            if self.is_alive():
                out = self.pose_guard(joint, movej = True)  
                if out:
                    return True, None, "setmovej 机械臂达位成功!"
                else:
                    return False, None, "setmovej 机械臂达位失败{Exception}_{e}!"
                
            return True,None, "setmovej 发送成功!"
        except Exception as e:
            return False,None, f"setmovej路点发送失败{Exception}_{e}"

    def setServol(self, xyzrxryrz, ms = 4):
        """
            走伺服l功能,10
            xyzrxryrz movel的点
            ms        走点的间隔1表示10ms 4表示40ms
        """
        try:
            x, y, z, rx, ry, rz = xyzrxryrz
            mess = "servol({"+f"{x},{y},{z},{rx},{ry},{rz}"+"},"+"0, 0,{},0,0)".format(ms)
            self.client_socket_.send(mess.encode())
            # return True,None,f"{mess}发送成功!"       # 由于servol需要实时性这里就不返回了
        except Exception as e:
            return False,None,f"[{mess}] movel发送失败{Exception}_{e}"
        
    def setTeachStart(self, num, dir):
        '''
        @description:
            手动示教开始指令,相当于示教器移动选项卡中单关节和轴操作
        @Args:
            num:1~6 关节号,表示关节移动;
                7~9 笛卡尔坐标系移动x,y,z;
                10~12 笛卡尔旋转Rx,Ry,Rz
            dir:0 代表反向移动
                1 代表正向移动
        @Return:
            标志位,None,状态说明反馈
        '''
        try:
            mess = f"robotTeachStart({num},{dir})"
            self.client_socket_.send(mess.encode())
            return True, None, f"{mess}发送成功!"
        except Exception as e:
            return False, None, f"[{mess}] 发送失败{Exception}_{e}"

    def setTeachEnd(self, num, dir):
        '''
        @description:
            手动示教结束指令,用来结束示教
        @Args:
            num:1~6 关节号,表示关节移动;
                7~9 笛卡尔坐标系移动x,y,z;
                10~12 笛卡尔旋转Rx,Ry,Rz
            dir:0 代表反向移动结束
                1 代表正向移动结束
        @Return:
            标志位,None,状态说明反馈
        '''
        try:
            mess = f"robotTeachEnd({num},{dir})"
            self.client_socket_.send(mess.encode())
            return True, None, f"{mess}发送成功!"
        except Exception as e:
            return False, None, f"[{mess}] 发送失败{Exception}_{e}"

    def setnutation(self,height,number,space = [80,80,80,5,0]):
        """点头"""
        '''
        @description:
            点头
        @Args:
            hight  要下降的高度(mm)
            numbel 下降次数
            space  速度
        @Return:
            标志位,None,状态说明反馈
        '''
        try:
            data = self.getmovel()[1]

            message = ""
            distance  = float(height/1000)
            if data is None:
                return False, None, f"获取位姿失败"
            
            x,y,z,rx,ry,rz = data
            zd = z - distance
            zs = z - 0.01
            num1,num2,num3,num4,num5 = space
            for i in range(number):
                message = message + "movel({"+f"{x},{y},{zd},{rx},{ry},{rz}"+"},"+f"{num1},{num2},{num3},{num4},{num5})\n"+"movel({"+f"{x},{y},{z},{rx},{ry},{rz}"+"},"+f"{num1},{num2},{num3},{num4},{num5})\n"
            mess = f"def\n{message}"+"movel({"+f"{x},{y},{zs},{rx},{ry},{rz}"+"},"+f"{num1},{num2},{num3},{num4},{num5})\n"+"end"

            self.client_socket_.send(mess.encode())
            
            if self.is_alive():
                points = [x, y, zs, rx, ry, rz]
                flag_out = self.pose_guard(arm_pose = points, movej=False, filter_fist= False)
                if flag_out:
                    return True, None, f"[setnutation] 达位成功"
                else:
                    return False, None, f"[setnutation] 达位失败"

            return True, None, f"[{mess}] 发送成功"
        except Exception as e:
            return False, None, f"[{mess}] 发送失败{Exception}_{e}"
          
    def settraject(self, points, speed=[90, 90, 90, 5, 0], movej=True):
        """轨迹交融"""
        '''
        @description:
            设置输出数字IO
        @Args:
            data movej指令,每个指令需要加\n换行
        @Return:
            标志位,None,状态说明反馈
        '''
        try:
            data = ""
            for i in range(len(points)):
                x,y,z,rx,ry,rz=points[i]
                num1,num2,num3,num4,num5 = speed
                if movej:
                    data = data + "movej({"+f"{x},{y},{z},{rx},{ry},{rz}"+"},"+f"{num1},{num2},{num3},{num4},{num5})\n"
                else:
                    data = data + "movel({"+f"{x},{y},{z},{rx},{ry},{rz}"+"},"+f"{num1},{num2},{num3},{num4},{num5})\n"
            mess = f"def\n{data}end"
            self.client_socket_.send(mess.encode())
            
            # 轨迹交融达位判断
            if self.is_alive():
                # (如果第一个点和最后一个点一样则不能进行判断)
                if points[0] == points[-1]:
                    if movej:
                        flag_out = self.pose_guard(arm_pose = points[-1], movej=True, filter_fist= True)
                    else:
                        flag_out = self.pose_guard(arm_pose = points[-1], movej=False, filter_fist= True)
                else:
                    if movej:
                        flag_out = self.pose_guard(arm_pose = points[-1], movej=True, filter_fist= False)
                    else:
                        flag_out = self.pose_guard(arm_pose = points[-1], movej=False, filter_fist= False)
                if flag_out:
                    return True, None, f"[settraject] 达位成功"
                else:
                    return False, None, f"[settraject] 达位失败"
                    
            return True, None, f"[{mess}] 未开启线程发送成功"
        except Exception as e:
            return False, None, f"[{mess}] 发送失败{Exception}_{e}"
    
    def settrajectl(self, point, point_num, Rir=0.1, cir=3, speed=[25, 80, 80, 5, 0]):
        """绕圈"""
        '''
        @description:
            设置输出数字IO
        @Args:
            point movel指令,每个指令需要加\n换行
        @Return:
            标志位,None,状态说明反馈
        '''
        try:
            data = ""
            radius = Rir
            point_number = point_num
            for i in range(point_number):
                angle = 2 * math.pi * i / point_number
                x = round(point[0] + radius * math.cos(angle),3)
                y = round(point[1] + radius * math.sin(angle),3)
                z,rx,ry,rz = round(point[2],3),round(point[3],3),round(point[4],3),round(point[5],3)
                num1,num2,num3,num4,num5 = speed
                data = data + "movel({"+f"{x},{y},{z},{rx},{ry},{rz}"+"},"+f"{num1},{num2},{num3},{num4},{num5})\n"
            data = data * int(cir)
            mess = f"def\n{data}end"
            self.client_socket_.send(mess.encode())
            return True, None, f"[{mess}] 发送成功"
        except Exception as e:
            return False, None, f"[{mess}] 发送失败{Exception}_{e}"
        
    def setDO(self, CHL, VALUE):
        '''
        @description:
            设置输出数字IO
        @Args:
            CHL:   通道号(15)
            VALUE: 0为低 1为高
        @Return:
            标志位,None,状态说明反馈
        '''
        try:
            mess = f"setDO({CHL},{VALUE})"
            self.client_socket_.send(mess.encode())
            return True, None, f"[{mess}] 发送成功"
        except Exception as e:
            return False, None, f"[{mess}] 发送失败{Exception}_{e}"
    
    # TODO 暂时不用
    def setAO(self, CHL, VALUE):
        '''
        @description:
            设置输出模拟IO
        @Args:
            CHL:   通道号
            VALUE: 为值-10~10V
        @Return:
            标志位,None,状态说明反馈
        '''
        try:
            mess = f"setAO({CHL},{VALUE})"
            self.client_socket_.send(mess.encode())
            return True, None, f"[{mess}] 发送成功"
        except Exception as e:
            return False, None, f"[{mess}] 发送失败{Exception}_{e}"

    def setWaitTime(self, second=3000):
        """等待时间"""
        try:
            mess = f"wait({second})"
            self.client_socket_.send(mess.encode())
            return True, None, f"[{mess}] 等待时间成功!"
        except Exception as e:
            return False, None, f"[{mess}] 等待时间失效{Exception}_{e}"
    
    def setToolBit(self, VALUE):
        '''
        @description:
            设置输出数字IO(康养测试过) NOTE 在机械臂running条件(状态12)下开启才有效
        @Args:
            VALUE: 0为低 1为高
        @Return:
            标志位,None,状态说明反馈
        '''
        try:
            mess = f"setToolBit(0,7,{VALUE})"
            self.client_socket_.send(mess.encode())
            return True, None, f"[{mess}] 发送成功"
        except Exception as e:
            return False, None, f"[{mess}] 发送失败{Exception}_{e}"
        
    #------------------------------------------------#
    #             获取机械臂位置                       #
    #------------------------------------------------#
    def help_func(self, command):
        """ 一个获取状态的帮助函数 """
        try:
            # 如果你开启了线程
            if self.is_alive():
                count = 0
                while True:
                    
                    # 获取机械臂线程回调数据
                    if self.get_cb_data() is None:
                        continue
                    mess_ = self.get_cb_data()
                    if mess_:
                        mess = mess_[command]
                        if command == "movej" or command == "movel":
                            # 获取坐标全0 [0,0,0,0,0,0]
                            if (np.array(mess)==0).all():
                                count +=1
                                continue
                            
                        return True, mess, f"get {command} success"
                    else:
                        count +=1
                    if count >5:
                        return False, None, f"get {command} failure"
                    time.sleep(0.05)

            else:
                count_num = 0
                while True:

                    # 获取机械臂线程回调数据
                    flag, mess_, _ = self.recv_mess_from_arm()
                    time.sleep(0.05)
                    if flag:
                        mess = mess_[command]
                        if command == "movej" or command == "movel":
                            # 获取坐标全0 [0,0,0,0,0,0]
                            if (np.array(mess)==0).all():
                                count_num +=1
                                continue
                            
                        return True, mess, f"get {command} success"
                    else:
                        count_num +=1
                    if count_num >5:
                        return False, None, f"get {command} failure"
        except Exception as e:
            return False, None, f"{Exception}_{e}"
               
    def getmovej(self):
        '''获取当前位姿'''
        return self.help_func(command= "movej")

    def getmovel(self):
        '''获取工具末端'''     
        return self.help_func(command= "movel")

    def getDi_status(self):
        '''获取io状态'''
        return self.help_func(command= "Di_status_list")

    # NOTE 暂时不对外开放
    def __getJoin_mode(self):
        '''获取关节模式'''
        return self.help_func(command= "join_mode_list")

    def getStatus(self):
        '''获取机器人状态'''
        return self.help_func(command= "status_list")

    def getIsArriving(self):
        '''获取达位信息'''
        return self.help_func(command= "IsArriving_list")

    def getToolIO(self):
        return self.help_func(command= "Tool_IO")

    def getListSize(self):
        return self.help_func(command= "List_Size")
        
    def getData(self):
        """ 重写多线程中的获取数据方法 """
        flag, data, mess_ = self.recv_mess_from_arm()
        if flag:
            # 上锁保证数据安全
            self.update_cb_data(data)
    
    # 二次封装的用法,模拟上抬和下降 
    # NOTE 后面可以用矩阵优化,这样就能走末端tcp坐标系的上下了
    def setuplift(self, distance, speed = 40):
        """
            机械臂上升
            distance 单位是mm
            speed 是会发送给setmovel的值
            1. 获取当前的movel
            2. 根据需要上升和下降的距离计算
        """
        try:
            if self.is_alive():
                while True:

                    if self.get_cb_data() is None:
                        time.sleep(0.05)
                        continue
                    movel = self.get_cb_data()["movel"]
                    break
            else:
                flag, data, info_ = self.recv_mess_from_arm()
                if flag:
                    movel = data["movel"]
                else:
                    return False, None, f"移动机械臂上升{distance}mm 失败,没有获取到当前机械臂位姿"
            new_movel = [movel[0], movel[1], movel[2] + (distance/1000), movel[3], movel[4], movel[5]]
            return self.setmovel(new_movel, speed)
        except Exception as e:
            return False, None, f"移动机械臂上升{distance}mm 失败{Exception}_{e}"
            
    def setdown(self, distance, speed =30):
        """
            机械臂下降 
            1. 获取当前的movel
            2. 根据需要上升和下降的距离计算
        """
        try:
            if self.is_alive():
                while True:
                    
                    if self.get_cb_data() is None:
                        time.sleep(0.05)
                        continue
                    movel = self.get_cb_data()["movel"]
                    break
            else:
                flag, data, info_ = self.recv_mess_from_arm()
                if flag:
                    movel = data["movel"]
                else:
                    return False, None, f"移动机械臂上升{distance}mm 失败,没有获取到当前机械臂位姿"
            new_movel = [movel[0], movel[1], movel[2] - (distance/1000), movel[3], movel[4], movel[5]]
            return self.setmovel(new_movel, speed)
        except Exception as e:
            return False, None, f"移动机械臂上升{distance}mm 失败{Exception}_{e}"
            
    #------------------------------------------------#
    #                 解析socket信息                  #
    #------------------------------------------------#
    def recv_mess_from_arm(self):
        '''接收来自客户端的信息'''
        try:
            #------修改时间 2023/4/25 清空机械臂缓存，获取实时位姿数据 --------
            # 一般来说，阻塞模式适合于简单的网络应用,例如聊天室或网页浏览器,
            # 而非阻塞模式则更适合处理大量并发连接或需要快速响应的网络应用，例如在线游戏或实时视频传输
            if self.__block_com is False and self.clear_socket:      # 在非堵塞模式下用
                clearSocketBuffer(self.client_socket_)
            client_data_ = self.client_socket_.recv(868)
            client_data = self.analyze_mess_from_arm(client_data_)
            return True, client_data, "数据接解析成功!"
        except Exception as e:
            return False,None, f"数据接收失败{Exception}_{e}!"

    def analyze_mess_from_arm(self, arm_data):
        '''解析表'''
        assert len(arm_data) == 868  # 判断接收数据的总长度
        
        # '''------1.分解字段-----'''
        # Len_ = arm_data[0:4]  # 数据长度

        #target_pos[4:4+48]                         目标位姿
        target_pos_1_ = arm_data[4:4+8]
        target_pos_2_ = arm_data[12:12+8]
        target_pos_3_ = arm_data[20:20+8]
        target_pos_4_ = arm_data[28:28+8]
        target_pos_5_ = arm_data[36:36+8]
        target_pos_6_ = arm_data[44:44+8]

        #target_v[52:52+48]                          #目标速度
        target_v_1_ = arm_data[52:52+8]
        target_v_2_ = arm_data[60:60+8]
        target_v_3_ = arm_data[68:68+8]
        target_v_4_ = arm_data[76:76+8]
        target_v_5_ = arm_data[84:84+8]
        target_v_6_ = arm_data[92:92+8]

        #target_acc[100:100+48]                      目标加速度
        target_acc_1_ = arm_data[100:100+8]
        target_acc_2_ = arm_data[108:108+8]
        target_acc_3_ = arm_data[116:116+8]
        target_acc_4_ = arm_data[124:124+8]
        target_acc_5_ = arm_data[132:132+8]
        target_acc_6_ = arm_data[140:140+8]

        #target_l[148:148+48]                        目标电流值
        target_l_1_ = arm_data[148:148+8]
        target_l_2_ = arm_data[156:156+8]
        target_l_3_ = arm_data[164:164+8]
        target_l_4_ = arm_data[172:172+8]
        target_l_5_ = arm_data[180:180+8]
        target_l_6_ = arm_data[188:188+8]

        #target_M[196:196+48]                        目标力矩
        target_M_1_ = arm_data[196:196+8]
        target_M_2_ = arm_data[204:204+8]
        target_M_3_ = arm_data[212:212+8]
        target_M_4_ = arm_data[220:220+8]
        target_M_5_ = arm_data[228:228+8]
        target_M_6_ = arm_data[236:236+8]

        #actual_pos[244:244+48]                      当前位姿
        actual_pos_1_ = arm_data[244:244+8]
        actual_pos_2_ = arm_data[252:252+8]
        actual_pos_3_ = arm_data[260:260+8]
        actual_pos_4_ = arm_data[268:268+8]
        actual_pos_5_ = arm_data[276:276+8]
        actual_pos_6_ = arm_data[284:284+8]

        # actual_v[292:292+48]                       当前速度
        actual_v_1_ = arm_data[292:292+8]
        actual_v_2_ = arm_data[300:300+8]
        actual_v_3_ = arm_data[308:308+8]
        actual_v_4_ = arm_data[316:316+8]
        actual_v_5_ = arm_data[324:324+8]
        actual_v_6_ = arm_data[332:332+8]

        # actual_l[340:340+48]                       当前电流值
        actual_l_1_ = arm_data[340:340+8]
        actual_l_2_ = arm_data[348:348+8]
        actual_l_3_ = arm_data[356:356+8]
        actual_l_4_ = arm_data[364:364+8]
        actual_l_5_ = arm_data[372:372+8]
        actual_l_6_ = arm_data[380:380+8]

        # 工具末端[388:388+48]
        tool_x_ = arm_data[388:388+8]  # 工具末端-X
        tool_y_ = arm_data[396:396+8]  # 工具末端-Y
        tool_z_ = arm_data[404:404+8]  # 工具末端-Z
        tool_rx_ = arm_data[412:412+8]  # 工具末端-Rx
        tool_ry_ = arm_data[420:420+8]  # 工具末端-Ry
        tool_rz_ = arm_data[428:428+8]  # 工具末端-Rz

        # tcp_force[436:436+48]                      工具力
        tcp_force_1_ = arm_data[436:436+8]
        tcp_force_2_ = arm_data[444:444+8]
        tcp_force_3_ = arm_data[452:452+8]
        tcp_force_4_ = arm_data[460:460+8]
        tcp_force_5_ = arm_data[468:468+8]
        tcp_force_6_ = arm_data[476:476+8]

        # tool_v[484:484+48]                         工具线速度
        tool_v_1_ = arm_data[484:484+8]
        tool_v_2_ = arm_data[492:492+8]
        tool_v_3_ = arm_data[500:500+8]
        tool_v_4_ = arm_data[508:508+8]
        tool_v_5_ = arm_data[516:516+8]
        tool_v_6_ = arm_data[524:524+8]

        # tcp_speed[532:532+48]                      工具坐标系速度矢量
        tcp_speed_1_ = arm_data[532:532+8]
        tcp_speed_2_ = arm_data[540:540+8]
        tcp_speed_3_ = arm_data[548:548+8]
        tcp_speed_4_ = arm_data[556:556+8]
        tcp_speed_5_ = arm_data[564:564+8]
        tcp_speed_6_ = arm_data[572:572+8]

        Di_status_ = arm_data[580:580+8]             # io状态

        tool_io_ = arm_data[588:588+8]               # 末端io

        # motor_temp[596:596+48]                     电机温度
        motor_temp_1_ = arm_data[596:596+8]
        motor_temp_2_ = arm_data[604:604+8]
        motor_temp_3_ = arm_data[612:612+8]
        motor_temp_4_ = arm_data[620:620+8]
        motor_temp_5_ = arm_data[628:628+8]
        motor_temp_6_ = arm_data[636:636+8]

        Controller_time_ = arm_data[644:644+8]       # 控制周期
        robot_mode_ = arm_data[652:652+8]            # 机器人模式

        # join_mode[660:660+48]                      关节模式
        join_mode_1_ = arm_data[660:660+8]
        join_mode_2_ = arm_data[668:668+8]
        join_mode_3_ = arm_data[676:676+8]
        join_mode_4_ = arm_data[684:684+8]
        join_mode_5_ = arm_data[692:692+8]
        join_mode_6_ = arm_data[700:700+8]

        run_time_ = arm_data[708:708+8]           # 程序运行时间
        status_   = arm_data[716:716+8]           # 机器人状态

        reserved_ = arm_data[724:724+120]         # 保留字段
        list_size = arm_data[844:844+8]           # 康养队列缓存大小

        IsArraiving_ = arm_data[852:852+8]        # 达位状态
        heartbit_ = arm_data[860:860+8]           # 心跳
        # '''------2.解析字段--------'''

        n_q = Struct('<d')                           # 8byte的十六进制转化为十进制,小端 带符号位
        n_B = Struct('B')                            # 无符号位的一个byte,十六进制转化为十进制
        n_i = Struct("i")                            # 4byte的十六进制转化为十进制

        # 当前位姿
        actual_pos_1 = n_q.unpack(actual_pos_1_)[0] 
        actual_pos_2 = n_q.unpack(actual_pos_2_)[0] 
        actual_pos_3 = n_q.unpack(actual_pos_3_)[0] 
        actual_pos_4 = n_q.unpack(actual_pos_4_)[0] 
        actual_pos_5 = n_q.unpack(actual_pos_5_)[0] 
        actual_pos_6 = n_q.unpack(actual_pos_6_)[0] 

        # 工具末端
        tool_x = n_q.unpack(tool_x_)[0] / 1000  # unpack()解析字节流,返回tuple类型
        tool_y = n_q.unpack(tool_y_)[0] / 1000
        tool_z = n_q.unpack(tool_z_)[0] / 1000
        tool_Rx = n_q.unpack(tool_rx_)[0]   
        tool_Ry = n_q.unpack(tool_ry_)[0]   
        tool_Rz = n_q.unpack(tool_rz_)[0]  

        # io状态
        Di_status = bin(int(n_q.unpack(Di_status_)[0]))
        Tool_IO   = int(n_q.unpack(tool_io_)[0])
        # # io状态---
        Di_status_bin = bin(int(n_q.unpack(Di_status_)[0]))[2:][::-1]
        if len(Di_status_bin) == 32:
            Di_status = [Di_status_bin[0:16],Di_status_bin[16:32]]
        if len(Di_status_bin) == 31:
            Di_status = [Di_status_bin[0:16],Di_status_bin[16:31]+"0"]

        # # 末端io
        # Tool_IO_int   = int(n_q.unpack(tool_io_)[0])
        # if Tool_IO_int == 3:
        #     Tool_IO = "00"
        # elif Tool_IO_int == 65539:
        #     Tool_IO = "10"
        # elif Tool_IO_int == 131075:
        #     Tool_IO = "01"
        # elif Tool_IO_int == 196611:
        #     Tool_IO = "11"

        # 关节模式
        join_mode_1 = n_q.unpack(join_mode_1_)[0] / 10000
        join_mode_2 = n_q.unpack(join_mode_2_)[0] / 10000
        join_mode_3 = n_q.unpack(join_mode_3_)[0] / 10000
        join_mode_4 = n_q.unpack(join_mode_4_)[0] / 10000
        join_mode_5 = n_q.unpack(join_mode_5_)[0] / 10000
        join_mode_6 = n_q.unpack(join_mode_6_)[0] / 10000

        # 机器人状态
        status = n_q.unpack(status_)[0]

        # 达位信息
        IsArriving = n_q.unpack(IsArraiving_)[0]
        List_Size  = n_q.unpack(list_size)[0]

        # '''--------3.字段存放列表----------'''
        actual_pos_list = list()
        actual_pos_list.append(actual_pos_1)
        actual_pos_list.append(actual_pos_2)
        actual_pos_list.append(actual_pos_3)
        actual_pos_list.append(actual_pos_4)
        actual_pos_list.append(actual_pos_5)
        actual_pos_list.append(actual_pos_6)

        tool_xyzrxryrz_list = list()
        tool_xyzrxryrz_list.append(tool_x)
        tool_xyzrxryrz_list.append(tool_y)
        tool_xyzrxryrz_list.append(tool_z)
        tool_xyzrxryrz_list.append(tool_Rx)
        tool_xyzrxryrz_list.append(tool_Ry)
        tool_xyzrxryrz_list.append(tool_Rz)

        Di_status_list = list()
        Di_status_list.append(Di_status)

        join_mode_list = list()
        join_mode_list.append(join_mode_1)
        join_mode_list.append(join_mode_2)
        join_mode_list.append(join_mode_3)
        join_mode_list.append(join_mode_4)
        join_mode_list.append(join_mode_5)
        join_mode_list.append(join_mode_6)

        status_list = list()
        status_list.append(status)

        IsArriving_list = list()
        IsArriving_list.append(IsArriving)

        # '''---------4.列表存入字典---------'''
        data = dict()
        data["movej"]           = [round(i, 3) for i in actual_pos_list]
        data["movel"]           = [round(i, 3) for i in tool_xyzrxryrz_list ]    
        data["Di_status_list"]  = Di_status_list[0]           # 数字输出
        # data["join_mode_list"]  = join_mode_list            # 暂时没使用
        
        # 0是未上电,1和3过渡点,2是上电成功,4是就绪状态,6手动示教,10暂停,12运行中,
        # 13拖动,16 起始点,17 运行中途获取新点 19 急停,20错误状态    
        data["status_list"]     = int(status_list[0]) 
        data["IsArriving_list"] = int(IsArriving_list[0])    # 达位状态
        data["Tool_IO"]         = Tool_IO                    # 末端按钮状态(取点,被动拖动示教无质量物体)
        data["List_Size"]       = int(List_Size)             # 康养队列缓存大小
        return data

    #------------------------------------------------#
    #                 辅助方法(堵塞式)                 #
    #------------------------------------------------#
    def reach_pose(self, pose1, pose2, movej = True):
        """
            达位判断,判断当前位置和目标位置
        """
        pose1 = np.array(pose1)
        pose2 = np.array(pose2)
        
        # 对xyz判断
        diff_pose_xyz = np.round(np.abs(pose1[:3] - pose2[:3]), 3)
        # 对rpy判断
        diff_pose_rxryrz = np.round(np.abs(pose1[3:] - pose2[3:]),3)
        
        if movej:
            # 走的关节
            reach_pose_xyz    = (diff_pose_xyz < 0.05).all()
            reach_pose_rxryrz = (diff_pose_rxryrz < 0.05).all()
        else:
            # 走的movej关节
            reach_pose_xyz    = (diff_pose_xyz < 0.001).all()
            reach_pose_rxryrz = (diff_pose_rxryrz < 0.05).all()   
        
        return reach_pose_xyz and reach_pose_rxryrz

    def reach_pose_flag(self):
        """
            根据当前的标志位进行判断
            armobject是机械臂的对象
        """
        temp_list = []      # 存放达位统计
        while True:
            
            outcome = self.get_cb_data()["IsArriving_list"]
            time.sleep(0.005)
            if outcome is None:
                time.sleep(0.05)
                continue
            
            temp_list.append(outcome)
            sum_flag = sum(temp_list)
            # print(temp_list)
            if sum_flag >= 1 and temp_list[-1] ==0 and temp_list[-2] ==0:
                temp_list.clear()
                return True
            if outcome == 1:
                time.sleep(0.05)
                continue
 
    def pose_guard(self, arm_pose, movej = True, filter_fist = False):
        """
        描述: 监听移动达位
        参数: arm_pose 目标机械臂位姿
        参数: movej 需要监听的指令是movel还是movej
        参数: filter_fist 是否过滤第一个点,这个主要用于轨迹交融又走回来第一个位置
        输出: bool
        """
        flag = 0
        while True:
            
            temp_arm_callback_data = self.get_cb_data()
            if temp_arm_callback_data is None:
                flag +=1
                continue
            elif flag > 1000:
                break
            else:
                break
            
        # 保证通信成功
        if temp_arm_callback_data is None:
            return False
        if movej:
            current_pose = temp_arm_callback_data["movej"]
        else:
            current_pose = temp_arm_callback_data["movel"]
        
        if filter_fist is False:
            # filter_fist 是否过滤第一个点,这个主要用于轨迹交融又走回来第一个位置
            reach_state1 = self.reach_pose(current_pose, arm_pose, movej)
            if reach_state1:
                return True
        reach_state2 = self.reach_pose_flag()
        if reach_state2:
            return True
        reach_state = self.reach_pose(current_pose, arm_pose, movej)
        return reach_state
     
    def waitArmStart(self, expect_state):
        """ 堵塞的等待机械臂的状态 """

        # 等待30秒
        if expect_state == 4:  # 上电20秒
            deadline = 400
        else:
            deadline = 200    # 其他的10秒
                
        flag = 0
        while True:
            try:
                outcome = self.get_cb_data()["status_list"]
            except Exception as e:
                outcome = None
            
            # 在运行状态
            if outcome == 12:
                return True

            if outcome is None:
                continue
            
            if outcome == expect_state:
                flag = 0
                return True
            elif outcome in [1,2,3]:
                flag =0

            if flag >=deadline:
                print("发生错误的flag值",flag)
                return False
            time.sleep(0.05)
            
        
if __name__ == "__main__":
    temp = iimtCobot('192.168.10.20',30003)
    com_out = temp.instanceSocket()
    print(com_out)
    
