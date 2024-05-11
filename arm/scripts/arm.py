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
    # 第一个线程的执行逻辑
    
    demo = armDemo()
    # 设置ip、port
    demo.ip   = "192.168.12.16"
    demo.port = 5033
    # 1.连接socket
    out = demo.instanceSocket()
    print(out)

    # 反馈信号 msg ，将arm_bus至false，使小车前往下一个点

    msg = Bool()
    cap_falg = UInt8()
    # demo.setFastPowerOn()
    # time.sleep(6)
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
                if arm_bus == 1 and point_bus!=8 and point_bus!=16 and point_bus!=17 and point_bus!=21 and point_bus!=27 and point_bus!=40:
                    cap_nums = point_data[point_bus]['cap_nums']

                    # print(cap_nums)
                    for i in range(cap_nums):
                        Move_fileName = point_data[point_bus]['cap_paths'][i]
                        focus = point_data[point_bus]['focus'][i]
                        change_zoom(focus)
                        rospy.loginfo("Start Move!")
                        # 3.1开启线程(监听达位信息需要开启线程)
                        # 3.2读取文件
                        print(Move_fileName)
                        flag,data,info = demo.read_record(Move_fileName)
                        # 发送位置
                        print("point_bus is: ", point_bus)
                        print(data)
                        for movej_ in data:
                            flag,data,info = demo.setmovej(movej_)
                            if flag:
                                print(info)
                                continue
                            else:
                                break
                        num = int(i)
                        if ((point_bus == 7 or point_bus == 26) and num==4) or (point_bus == 25 and num == 3) or (point_bus == 18 and num == 2):
                            arm_bus = 0
                            msg.data = True
                            pub.publish(msg)
                            time.sleep(0.5)
                            break
                        rospy.loginfo("Start Wave!")
                        Wave_file_path = '/home/dell/catkin_ws/src/arm/scripts/capture.txt'
                        out = demo.directionPlus(bias=0.10427,joint=5)
                        flag = 1
                        if os.path.exists(Wave_file_path):
                            if os.stat(Wave_file_path).st_size == 0:
                                with open(Wave_file_path, "w") as f:
                                    f.write("1 ")
                                    f.write(str(cap_bus+1))
                                    cap_bus+=1
                                    print("Set 1, start capture!")
                                    time.sleep(2)
                        else:
                            print(f"{Wave_file_path} not exist")
                        while flag != 0 and not rospy.is_shutdown():
                            if os.stat(Wave_file_path).st_size == 0:
                                out = demo.directionMinus(0.034907, joint=5)
                                with open(Wave_file_path, "w") as f:
                                    f.write("1")
                                    flag += 1
                                if(flag == 7):
                                    flag = 0
                            time.sleep(2)
                        out = demo.directionPlus(bias=0.10427,joint=5)
                        rospy.loginfo("Capture Done!")
                        time.sleep(2)

                        """
                        留给摄像头拍摄的接口

                        time.sleep(0.2)
                        
                        """
                        cap_falg.data = UInt8(i)
                        cap_pub.publish(cap_falg)
                        time.sleep(0.2)
                        rospy.loginfo("Done!")

                    arm_bus = 0
                    msg.data = True
                    pub.publish(msg)
                    time.sleep(0.5)
                if  point_bus == 8 or point_bus == 16 or point_bus == 17 or point_bus == 21 or point_bus ==27:
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
    # 建立point_data存储各检测点位拍摄数量和机械臂运动轨迹文件位置
    point_data = {}
    
    # 接收arm_bus信号并检测上升沿
    arm_bus_status = False
    arm_bus_last_status = False
    # 机械臂控制总线，用于逻辑控制
    arm_bus = 0
    # 点位总线，用于逻辑控制
    point_bus = 1
    # 途径点
    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_1.txt"]
    focus = [3.1]
    point_data[1] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus': focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_1.txt",
                             "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_2.txt"]
    focus = [3.1,1.3]
    point_data[2] = {'cap_nums': 2, 'cap_paths': cap_paths, 'focus': focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_3.txt"]
    focus = [1.0]
    point_data[3] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_4.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_5.txt"]
    focus = [1.0, 1.8]
    point_data[4] = {'cap_nums': 2, 'cap_paths': cap_paths, 'focus':focus}


    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_6.txt", 
                            "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_7.txt",
                            "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_8.txt" ]
    focus = [1.4,1.4, 1.0]
    point_data[5] = {'cap_nums': 3, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_9.txt", 
                               "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_10.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_11.txt" ]
    focus = [1.4, 1.2, 2.9]
    point_data[6] = {'cap_nums': 3, 'cap_paths': cap_paths, 'focus':focus}

    #点7 最后一个不用拍照
    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_12.txt", 
                               "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_13.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_14.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_15.txt",
                                 "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_16.txt" ]
    focus = [3.9, 2.0, 3.1,1.5, 1.0]
    point_data[7] = {'cap_nums': 5, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_16.txt"]
    focus = [3.1]
    point_data[8] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus': focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_16.1.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_16.txt"]
    focus = [1.4, 1.0]
    point_data[9] = {'cap_nums': 2, 'cap_paths': cap_paths, 'focus':focus}


    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_16.txt"]
    focus = [1.0]
    point_data[10] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_16.txt"]
    focus = [1.0]
    point_data[11] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_16.txt"]
    focus = [1.0]
    point_data[12] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_16.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_16.2.txt"]
    focus = [1.0, 1.0]
    point_data[13] = {'cap_nums': 2, 'cap_paths': cap_paths, 'focus':focus}



    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_16.txt"]
    focus = [1.0]
    point_data[14] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_17.txt"]
    focus = [2.0]
    point_data[15] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_1.txt"]
    focus = [3.1]
    point_data[16] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus': focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_1.txt"]
    focus = [3.1]
    point_data[17] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus': focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_18.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_19.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_1.txt"]
    focus = [1.0, 1.0,1.0]
    point_data[18] = {'cap_nums': 3, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_21.txt"]
    focus = [1.3]
    point_data[19] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_22.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_23.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_23.5.txt"]
    focus = [1.0, 2.1,1.0]
    point_data[20] = {'cap_nums': 3, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_1.txt"]
    focus = [3.1]
    point_data[21] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus': focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_24.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_25.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_30.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_31.txt"]
    focus = [2.1, 2.1,2.0,1.1]
    point_data[22] = {'cap_nums': 4, 'cap_paths': cap_paths, 'focus':focus}

    #配准1
    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_26.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_27.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_28.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_29.txt"]
    focus = [1.6, 1.6,3.6, 3.3]
    point_data[23] = {'cap_nums': 4, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_32.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_33.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_34.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_35.txt"]
    focus = [1.5, 2.29, 2.4, 2.4]
    point_data[24] = {'cap_nums': 4, 'cap_paths': cap_paths, 'focus':focus}

    #点25 最后一个不用拍照
    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_36.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_37.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_38.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_1.txt"]
    focus = [1.0, 3.5, 1.0, 1.0]
    point_data[25] = {'cap_nums': 4, 'cap_paths': cap_paths, 'focus':focus}

    #点26 最后一个不用拍照
    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_39.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_40.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_41.5.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_41.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_1.txt"]
    focus = [1.0, 2.8, 1.0, 1.0, 1.0]
    point_data[26] = {'cap_nums': 5, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_1.txt"]
    focus = [3.1]
    point_data[27] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus': focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_42.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_43.txt"]
    focus = [2.38,1.0]
    point_data[28] = {'cap_nums': 2, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_43.txt"]
    focus = [1.0]
    point_data[29] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_43.txt",
                            "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_43.1.txt"]
    focus = [1.0, 1.0]
    point_data[30] = {'cap_nums': 2, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_43.txt"]
    focus = [1.0]
    point_data[31] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_43.txt"]
    focus = [1.0]
    point_data[32] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_43.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_44.txt" ]
    focus = [1.0, 1.1]
    point_data[33] = {'cap_nums': 2, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_43.2.txt"]
    focus = [1.7]
    point_data[34] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus': focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_45.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_46.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_47.txt",
                                 "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_48.txt" ]
    focus = [2.2, 2.2, 1.5, 1.0]
    point_data[35] = {'cap_nums': 4, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_49.txt"]
    focus = [2.6]
    point_data[36] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_50.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_51.txt"]
    focus = [1.0, 1.2]
    point_data[37] = {'cap_nums': 2, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_52.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_53.txt",
                                "/home/dell/catkin_ws/src/arm/scripts/Point_path/move_54.txt"]
    focus = [1.4, 1.0, 3.9]
    point_data[38] = {'cap_nums': 3, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_55.txt"]
    focus = [1.4]
    point_data[39] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    cap_paths = ["/home/dell/catkin_ws/src/arm/scripts/Point_path/move_1.txt"]
    focus = [1.0]
    point_data[40] = {'cap_nums': 1, 'cap_paths': cap_paths, 'focus':focus}

    # time.sleep(2)
    point_bus_sub = rospy.Subscriber("/point_bus", UInt8, point_bus_Callback, queue_size = 5)
    # time.sleep(2)
    arm_sub= rospy.Subscriber("/arm_bus", Bool, arm_bus_Callback, queue_size = 20)
    # arm_sub= rospy.Subscriber("/set_arm_pose", Bool, arm_bus_Callback, queue_size = 20)

    t1 = threading.Thread(target=thread_function1)
    # 设置为守护线程（可选）
    t1.daemon = False
    t1.start()

    rate = rospy.Rate(10)
    print("Init done!")
    rate.sleep()
    # rospy.spin()
        