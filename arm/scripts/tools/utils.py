#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
.. 文件路径: ~/Codepy/eyeHand2/tools/utils.py'
.. 创建时间: 19:29:34
.. 作   者: 夏 炎
.. 联系方式: 邮箱2440212215@qq.com 微信:18261953682
.. 版本  号: 0.1
.. 开源协议: BSD
.. 编程语言: python3.8 低于这个版本的python运行可能会报错
.. 描   述: 
..      工具模块       
"""

import numpy as np
from math import *
import ujson as json      # C++改写的json如果你安装不上直接换成原始json
import cv2
import yaml
import time
import datetime
import select

def getHighLow(data:int):
    """
    根据输入数据获取高低位,data需要在[0, 20000]之间,但这里不做限制
    """
    assert isinstance(data,int), "输入的int_v必须是int类型现在的类型是{}".format(type(data))
    assert 0 <= data <= 65536, "目前只支持[0,65536]的转换,你当前输出值为{}".format(data)
    # 原始数据二进制去除 0b
    bin_data = bin(data)[2:]
    len_bin  = len(bin_data)

    # print("原始数据",bin_data)
    # print("原始数据长度:",len_bin)
    if len_bin <= 8:
        h_16 = hex(0x00)
    else:
        h_8  = bin_data[0:len(bin_data)-8]  # 获取高8位的二进制数字
        h_16 = hex(int(h_8, 2))             # 转换成16进制

    # 获取低8位的二进制数字
    l_8  = bin_data[len(bin_data)-8:]
    l_16 = hex(int(l_8, 2)) 

    return  h_16, l_16


def HighLow2Int(high, low):
    """
    高8位和低8位的数字转换成int类型
    """ 
    # 转换成16进制
    high_ = hex(high)
    low_  = hex(low)
    data_ = "0b" + bin(int(high_,16))[2:].zfill(8) + bin(int(low_ ,16))[2:].zfill(8)
    return int(data_,2)


def read_yaml(file):
    """更新配置文件"""
    with open(file, encoding="UTF-8") as fp:
        return yaml.load(fp, Loader=yaml.FullLoader)

def get_time_stamp():
    """获取时间戳用做id"""
    return round(time.time()*1000)

def json_respond_send(response, component, message, id_num=None,error = None):
    """
    @brief           服务器广播数据发送给客户端
    @param id        业务编号由client提供
    @param response  回执
    @param component 组件
    @param message   消息内容(服务器返回给客户端)
    @return json_data
    """        
    if id_num is None:
        obj = {"id": int(get_time_stamp()),
               "response": response,
               "component": component,
               "message": message,
               "errorcode":error
               }
    else:
        obj = {"id": int(id_num),
               "response": response,
               "component": component,
               "message": message,
               "errorcode":error
               }
    json_data = json.dumps(obj, ensure_ascii= True)  
    return json_data


#用于根据欧拉角计算旋转矩阵
def myRPY2R_robot(x, y, z):
    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz@Ry@Rx     # 先绕x轴 再绕y轴 最后绕z轴
    # R = Rx@Ry@Rz
    return R


#用于根据位姿计算变换矩阵
def pose_robot(x, y, z, Tx, Ty, Tz, use_radian = False, inv = False):
    if use_radian:
        thetaX = Tx / 180 * pi
        thetaY = Ty / 180 * pi
        thetaZ = Tz / 180 * pi
    else:
        thetaX = Tx
        thetaY = Ty
        thetaZ = Tz

    R = myRPY2R_robot(thetaX, thetaY, thetaZ)
    t = np.array([[x], [y], [z]])
    RT1 = np.column_stack([R, t])                    # 列合并
    RT1 = np.row_stack((RT1, np.array([0,0,0,1])))

    if inv:
        RT1 = np.linalg.inv(RT1)                     # 用于眼在手外
    return RT1


#用来从棋盘格图片得到相机外参
def get_RT_from_chessboard(img_path, chess_board_x_num, chess_board_y_num, K, D, chess_board_len):
    '''
    :param img_path         : 读取图片路径
    :param chess_board_x_num: 棋盘格x方向格子数
    :param chess_board_y_num: 棋盘格y方向格子数
    :param K                : 相机内参
    :param D                : 相机畸变系数
    :param chess_board_len  : 单位棋盘格长度,m
    :return                 : 相机外参
    '''
    
    img  = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 

    # 例寻找角点 11*8的网格角点54个 corners (54, 1, 2)
    ret, corners = cv2.findChessboardCorners(gray, (chess_board_x_num, chess_board_y_num), None)
    
    # 将角点的转换成[2,54]的数据格式,方便后续的运算
    corner_points = np.zeros((2, corners.shape[0]),dtype=np.float64)  
    for i in range(corners.shape[0]):
        corner_points[:, i] = corners[i, 0,:]

    # 处理下棋盘的真实物理尺寸, Z轴为0
    object_points = np.zeros((3, chess_board_y_num * chess_board_x_num), dtype=np.float64)  # [3, 54]
    flag = 0
    for i in range(chess_board_y_num):      
        for j in range(chess_board_x_num): 
            object_points[:2, flag] = np.array([i * chess_board_len, j * chess_board_len])   # 2cm的格子物理尺寸
            flag+=1

    # rvec, tvec 计算出 标定板坐标系 ==> 相机坐标系 R T
    retval, rvec, tvec  = cv2.solvePnP(object_points.T, corner_points.T, K, distCoeffs=D)

    # cv2.Rodrigues 旋转矩阵和旋转向量之间的相互转化
    RT = np.column_stack(((cv2.Rodrigues(rvec))[0],tvec))
    RT = np.row_stack((RT, np.array([0, 0, 0, 1])))

    return RT


#------------------------------------------------#
#                 协议生成装饰器                   #
#------------------------------------------------#
def class_inspector(cls):
    """
    一个装饰器函数,用于检查类的方法和参数,并将检查结果保存为 JSON 文件.
    """
    # 创建一个空列表,用于存储方法信息
    methods_info = []

    # 遍历类的方法
    for name, method in cls.__dict__.items():
        if callable(method) and not name.startswith("_"):
            # 获取参数名称和默认值
            param_names = list(method.__annotations__.keys()) if method.__annotations__ else []
            param_defaults = method.__defaults__ or [None] * len(param_names)
            params_info = dict(zip(param_names, param_defaults))

            # 将方法信息添加到列表中
            methods_info.append({
                "id": "01234567890",
                "level": "normal",
                "component": cls.__name__,
                "command": name,
                "params": params_info,
                "doc": method.__doc__.strip() if method.__doc__ else None  # 去除开头和结尾的空格和换行符
            })

    # 将结果写入与类同名的 JSON 文件中
    filename = f"{cls.__name__}.json"
    with open(filename, "w") as f:
        json.dump(methods_info, f, indent=4, ensure_ascii= False)

    return cls

#------------------------------------------------#
#                 协程装饰器                       #                   
#------------------------------------------------#
import asyncio
# 定义一个装饰器函数,将同步方法转换为异步方法
def async_method(func):
    async def wrapper(self, *args, **kwargs):
        return await asyncio.to_thread(func, self, *args, **kwargs)
    return wrapper


def D_matrix(matrix1,matrix2):
    """计算旋转矩阵之间的差值 初始位置和终点位置"""
    return np.dot(matrix1.T,matrix2)

def rotationMatrixToEulerAngles(R) :
    """旋转矩阵转欧拉角"""
    sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = atan2(R[2,1] , R[2,2])
        y = atan2(-R[2,0], sy)
        z = atan2(R[1,0], R[0,0])
    else :
        x = atan2(-R[1,2], R[1,1])
        y = atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

 
def D_time(func):  
    """ 定义一个计时器，传入一个函数，返回另一个附加了计时功能的方法 """    
    # 定义一个内嵌的包装函数，给传入的函数加上计时功能的包装  
    def wrapper(*args):  
        t1 = datetime.datetime.strptime(datetime.datetime.now().strftime('%H:%M:%S.%f'), '%H:%M:%S.%f')
        out = func(*args)  
        t2   = datetime.datetime.strptime(datetime.datetime.now().strftime('%H:%M:%S.%f'), '%H:%M:%S.%f')
        D_time = (t2 - t1).microseconds / 1000
        return list(out)+[D_time]
    # 将包装后的函数返回  
    return wrapper  

def recordData(file_name = "record",*args):
    """记录获取相机、机械臂的时间,以及计算出新位姿的时间"""
    data = ""
    for i in range(len(args)):
        data += f"data{i+1}:{args[i]}   "
        if i == len(args) - 1:
            data += "\n"
    with open(f"./{file_name}.txt",'a') as fp:
        fp.write(data)

def clearSocketBuffer(socket_client):
    while True:
        ready = select.select([socket_client], [], [], 0.01)
        if ready[0]:
            data = socket_client.recv(4)
        else:
            # print('超时清空')
            return True