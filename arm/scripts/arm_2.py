#!/usr/bin/env python3
#coding=utf - 8

import rospy

import threading

from std_msgs.msg import Bool
from std_msgs.msg import UInt8
import os
import re
import time
import warnings
from get_ptz import change_zoom
# warnings.filterwarnings("ignore")

from tools.iimtcobot import iimtCobot

class armDemo(iimtCobot):
    
    def __init__(self, ip: str = "127.0.0.1", port: int = 5033, thread_time: float = 0.3, block_com: bool = False):
        super().__init__(ip, port, thread_time, block_com)

        
    def movej_record(self,fileName = "movej"):
        '''
        @description:
            记录movej
        @Args:
            fileName: 保存路径,默认./movej.txt
        @Return:
            标志位,None,状态说明反馈
        '''
        try:
            flag,data,info = self.getmovej()
            time.sleep(0.01)
            if flag:
                flag_,data_,info_ = self.write_record(record=data, fileName=fileName)
                print(data)
                return flag_,data_,info_
            else:
                return False,None,info
        except Exception as e:
            return False,None,f"{Exception}_{e}"
    
    def directionPlus(self, bias = 0.08, joint = 5):
        """正方向移动bias"""
        assert joint in range(1,7)
        flag,movej,info = self.getmovej()
        time.sleep(0.01)
        if flag:
            movej[joint-1] += bias
            flag_,data_,info_ = self.setmovej(movej) 
            return flag_,data_,info_
        else:
            return False,None,info
    
    def directionMinus(self, bias = 0.08, joint = 5):
        """反方向移动bias"""
        assert joint in range(1,7)
        flag,movej,info = self.getmovej()
        time.sleep(0.01)
        if flag:
            movej[joint-1] -= bias 
            flag_,data_,info_ = self.setmovej(movej) 
            return flag_,data_,info_
        else:
            return False,None,info
        
    def write_record(self,record,fileName = "movej"):
        """将内容写进本地txt(行写入)"""
        try:
            if record == None:
                return False,None,"保存内容为空!"
            else:
                record = str(record)
                ff = open(f'./{fileName}.txt','a+')  #打开一个文件，可写模式
                ff.write(record+"\r")                #写入一个新文件中    
                ff.close()
                return True,None,"内容保存成功！"
        except Exception as e:
            return False,None,f"{Exception}_{e}"
    
    def read_record(self,fileName = "movej"):
        """读取txt文件(行读取)"""
        try:
            record_list = list() 
            ff = open(fileName, 'r')

            # ff = open(f'./{fileName}.txt','r')
            data = ff.readlines()
            ff.close()
            for record_ in data:
                record_ = record_.replace("\n","")
                record_ = re.findall(r'[[](.*?)[]]', record_)[0]
                record_ = record_.split(",")
                record_ = [float(var) for var in record_]
                record_list.append(record_)
            return True,record_list,None
        except Exception as e:
            return True,None,f'{Exception}_{e}'
        
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
         
def arm_bus_Callback(arm):

    global arm_bus, arm_bus_status, arm_bus_last_status
    global msg
    
    '''arm平时置False，抵达后置1启动机械臂'''
    if arm.data == True or arm.data == False:
        arm_bus_status = arm.data

    if (arm_bus_status and arm_bus_status != arm_bus_last_status):
        arm_bus = 1
        print("arm_bus set 1")
    if ((not arm_bus_status) and arm_bus_status != arm_bus_last_status):
        arm_bus = 0
        print("arm_bus set 0")
    arm_bus_last_status = arm_bus_status
 


        

def point_bus_Callback(point):
    global point_bus
    # print(point.data)
    point_bus = point.data

def thread_function1():
    demo = armDemo()
    demo.ip   = "192.168.12.16"
    demo.port = 5033
    out = demo.instanceSocket()
    print(out)
    msg = Bool()
    global arm_bus
    global point_bus
    global cap_bus
    cap_bus = 0
    global Move_fileName, Back_fileName
    pub = rospy.Publisher("/arm_feedback", Bool, queue_size = 10)  
    cap_pub = rospy.Publisher("/cap_bus", UInt8, queue_size = 10)
    demo.start()
    while True and not rospy.is_shutdown():
        while arm_bus != 0 and not rospy.is_shutdown():
                print("arm_bus:",arm_bus)
                if arm_bus == 1 and point_bus==3 or point_bus == 1:
                    cap_nums = point_data[point_bus]['cap_nums']
                    for i in range(cap_nums):
                        Move_fileName = point_data[point_bus]['cap_paths'][i]
                        focus = point_data[point_bus]['focus'][i]
                        change_zoom(focus)
                        rospy.loginfo("Start Move!")
                        print(Move_fileName)
                        flag,data,info = demo.read_record(Move_fileName)
                        print("point_bus is: ", point_bus)
                        print(data)
                        for movej_ in data:
                            flag,data,info = demo.setmovej(movej_)
                            if flag:
                                print(info)
                                continue
                            else:
                                break
                    arm_bus = 0
                    msg.data = True
                    pub.publish(msg)
                    time.sleep(0.5)
                
                arm_bus = 0
                msg.data = True
                pub.publish(msg)
                time.sleep(0.5)
                pub.publish(msg)
        msg.data = False
        pub.publish(msg)

if __name__ == "__main__":

    rospy.init_node("arm_node")
    rospy.loginfo("Arm node init!")

    global Move_fileName, Back_fileName
    global arm_bus, point_bus, arm_bus_status, arm_bus_last_status

    global point_data, cap_nums, cap_paths
    point_data = {}
    arm_bus_status = False
    arm_bus_last_status = False
    arm_bus = 0
    point_bus = 1

    # 途径点
    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_1.txt"]
    focus = [1.0]
    point_data[1] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus': focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_0.txt"]
    focus = [1.0]
    point_data[3] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    point_bus_sub = rospy.Subscriber("/point_bus", UInt8, point_bus_Callback, queue_size = 5)

    arm_sub= rospy.Subscriber("/arm_bus", Bool, arm_bus_Callback, queue_size = 20)

    t1 = threading.Thread(target=thread_function1)

    t1.daemon = False
    t1.start()

    rate = rospy.Rate(10)
    print("Init done!")
    rate.sleep()
        