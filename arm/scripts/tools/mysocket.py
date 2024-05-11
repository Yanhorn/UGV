#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
.. 文件路径: ~/Codepy/wellness/tools/mysocket.py'
.. 创建时间: 08:51:28
.. 作   者: 夏 炎
.. 联系方式: 邮箱2440212215@qq.com 微信:18261953682
.. 版本  号: 0.1
.. 开源协议: BSD
.. 编程语言: python3.8 低于这个版本的python运行可能会报错
.. 描   述: 
..      socket 通信的封装,其他使用模块只要继承这个即可
"""

import time
import socket
from tools.mythread import myThread

"""
 @brief 定义一个socket的工具其他的类将要通过继承的方式实现自己的功能 
 @param ip 连socket的ip地址
 @param port    连socket的端口号
 @param timeout 线程while循环的休眠时间
 @param block_mode 线程while循环的休眠时间
"""

class socketTools(myThread):
    def __init__(self, ip: str = "127.0.0.1",
                 port: int     = 8080,
                 thread_time: float = 0.3,
                 block_com: bool    = False):
        super().__init__(thread_time)

        self.__ip   = ip
        self.__port = port

        self.client_socket_   = None          # 定义socket的通信句柄
        self.__connect_timeout = 1            # 堵塞超时阈值
        self.__block_com       = block_com    # True:阻塞模式 False:非阻塞模式
    
    #------------------------------------------------#
    #               socket属性处理                    #
    #------------------------------------------------#
    @property
    def ip(self):
        return self.__ip
    
    @ip.setter
    def ip(self, new_ip):
        if type(new_ip) not in [str]:
            return False, None, f"new_ip must be str, but get type is {type(new_ip)}"
        self.__ip = new_ip
        return True, None, f"set {new_ip} success"

    @property
    def port(self):
        return self.__port
    
    @port.setter
    def port(self, new_port):
        if type(new_port) not in [int]:
            return False, None, f"new port must be int, but get type is {type(new_port)}"
        self.__port = new_port
        return True, None, f"set {new_port} success!"
    
    @property   
    def connect_timeout(self):
        return self.__connect_timeout
    
    @connect_timeout.setter
    def connect_timeout(self, time_out):
        if type(time_out) not in [int,float]:
            return False , None, f"new port must be int or float, but get type is {type(time_out)}"
        self.__connect_timeout = time_out
        return True, None, f"set {time_out} success!"
    
    @property
    def block_com(self):
        return self.__block_com

    def block_com(self, bool_flag):
        if type(bool_flag) not in [bool]:
            return False, None, f"bool flag must be bool, but get type is {type(bool_flag)}"
        self.__block_com = bool_flag
        return True, None, f"set {bool_flag} success!"
    
    #------------------------------------------------#
    #                  初始化方法                      #
    #------------------------------------------------#
    def instanceSocket(self):
        """创建网口实例化"""
        if self.client_socket_ is not None:
            self.stop()
            time.sleep(0.05)                       # 休眠50ms

        # 异常捕获 连接机械臂
        try:
            if self.__block_com:    # 阻塞模式
                self.client_socket_  =  socket.socket()
                self.client_socket_.connect((self.__ip, self.__port))
            else:                   # 非阻塞模式
                self.client_socket_  =  socket.socket() 
                self.client_socket_.setblocking(False)
                self.client_socket_.settimeout(self.__connect_timeout) # 超时时间  
                self.client_socket_.connect((self.__ip, self.__port))
                
                

            # 将running设置为True
            self.running = True       
            
            return True, None, "iimt arm instanceSocket success!"
        except Exception as e :
            return False,None, f"iimt arm instanceSocket failure {Exception}_{e}!"


if __name__ == '__main__':
    pass
    # 测试
    socket_arm = socketTools()
    print(socket_arm.ip)
    socket_arm.ip = "192.168.10.20"
    print(socket_arm.ip)
    
    print(socket_arm.port)
    socket_arm.port = 30003
    print(socket_arm.port)
    
    # 线程是否开启
    print(socket_arm.running)
    socket_arm.pause()
    print(socket_arm.flag.isSet())
    
    # 实例化是否成功
    out = socket_arm.instanceSocket()
    print(out)

