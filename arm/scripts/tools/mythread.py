#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
.. 文件路径: ~/Codepy/wellness/tools/mythread.py'
.. 创建时间: 14:37:52
.. 作   者: 夏 炎
.. 联系方式: 邮箱2440212215@qq.com 微信:18261953682
.. 版本  号: 0.1
.. 开源协议: BSD
.. 编程语言: python3.8 低于这个版本的python运行可能会报错
.. 描   述: 
..      python的线程工具   
..      通过threading.Event()可以创建一个事件管理标志,该标志(event)默认为False,event对象主要有四种方法可以调用
..      event.wait(timeout=None):调用该方法的线程会被阻塞,如果设置了timeout参数,超时后,线程会停止阻塞继续执行；
..      event.set()             :将event的标志设置为True,调用wait方法的所有线程将被唤醒
..      event.clear()           :将event的标志设置为False,调用wait方法的所有线程将继续被阻塞
..      event.isSet()           :判断event的标志是否为True
"""


import threading
import time

"""
 @brief    采用继承的方式实现线程 
 @attribute thread_time <float> : 线程的循环延迟时间   
 @NOTE  start只能执行一次,并且不能触发异常,即使stop也不能再次开始
        因为 threads can only be started once 所以解决办法也就很清晰了,重新创建一个对象

        def run(self):
                for each in self.threadList:
                    each.start()
                while True:
                    for a in xrange(5):
                        if not self.threadList[a].isAlive():
                            self.threadList[a] = Thread(a)
                            self.threadList[a].start()
                        sleep(3600) # 每个小时判断一下
"""
class myThread(threading.Thread):
    def __init__(self, thread_time:float = 0.01):
        super().__init__()

        self.__thread_time  = thread_time
        
        self.__flag    = threading.Event()         # 用于暂停线程的标识
        self.__flag.set()                          # 设置为True
        
        self.__running = threading.Event()         # 用于停止线程的标识
        self.__running.set()                       # 将running设置为True
    
    @property
    def thread_time(self):
        """ 获取线程时间 """
        return self.__thread_time
    
    @thread_time.setter
    def thread_time(self, new_time):
        """ 设置线程时间 """
        if type(new_time) not in [float, int]:
            return False, None, f"thread time must be int or float, but get type is {type(new_time)}!"
        self.__thread_time = new_time
        return True, None, f"thread set {new_time} success!"
    
    #------------------------------------------------#
    #                对线程进行操作                    #
    #------------------------------------------------#
    @property
    def running(self):
        """将running设置为True"""
        return self.__running
    
    @running.setter
    def running(self, flag):
        """将running设置为False"""
        if flag:
            self.__running.set()
        else:
            self.__running.clear()
            
    @running.getter
    def running(self):
        """判断running是否被设置为True"""
        return self.__running.isSet()
    
    @property
    def flag(self):
        """堵塞的标志位"""
        return self.__flag

    @flag.setter
    def flag(self, flag):
        """修改线程堵塞的状态 True不堵塞, False是堵塞"""
        if flag:
            self.__flag.set()
        else:
            self.__flag.clear()
    
    # 弃用 注意这里我并没有把它完全写成属性,因为继承的时候需要wait方法
    # @flag.getter
    # def flag(self):
    #     """判断flag True表示停止堵塞状态, False是表示堵塞状态"""
    #     return self.__flag.isSet()

    #------------------------------------------------#
    #                  实操用的较多接口                 #
    #------------------------------------------------#
    def getData(self):
        """不停的处理数据,一般子类继承使用"""
        print("我是测试的getData")
        pass

    def run(self):
        while self.__running.isSet():
            # 为True时立即返回,为False时阻塞直到内部的标识位为True后返回
            self.__flag.wait()
            
            # 不断的运行这个函数                   
            self.getData()
            
            # 线程刷新时间
            time.sleep(self.__thread_time)     
 
    def pause(self):
        """ 设置为False,让线程阻塞 """
        self.__flag.clear()             
 
    def resume(self):
        """ 设置为True,让线程停止阻塞 """
        self.__flag.set()       
    
    # 尽量不要用
    def stop(self):
        """ 停止线程 """
        self.__flag.set()        # 将线程从暂停状态恢复,如何已经暂停的话
        self.__running.clear()   # 设置为False  


if __name__ == "__main__":
    # 以下是功能测试代码
    import time
    
    # 继承的线程如何能够通过使用running和flag实现中断继续
    # NOTE running一般都是True,但是堵塞的flag就是有可能False或者True了
    class test(myThread):
        def __init__(self, thread_time: float = 0.01):
            super().__init__(thread_time)
        
        # NOTE 外部继承使用
        def run(self):
            while self.running:
                # 为True时立即返回,为False时阻塞直到内部的标识位为True后返回
                self.flag.wait()
                
                # 不断的运行这个函数                   
                self.getData()
                
                # 线程刷新时间
                time.sleep(self.thread_time)    

    th = test()
    th.thread_time = 0.1
    
    # print(th.thread_time)
    # print(th.running)
    # print(th.flag.isSet())
    
    print("开始")
    th.start()
    time.sleep(3)
    
    print("暂停")
    th.pause()
    time.sleep(3)
    
    print("开始")
    th.resume()
    time.sleep(3)
    
    print("暂停")
    th.pause()
