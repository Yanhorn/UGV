# -*- coding：utf-8 -*-
# get_ptz.py
# jn10010537

import requests
from requests.auth import HTTPDigestAuth
import time

def get_ptz(ip= "192.168.1.64",port =80,admin=  'admin',passwd= 'fh123456DH'):
    '''
    要求IPC有云台，否则没有ptz信息
    获取IPC云台的ptz位置信息
    '''
    res=None

    url = f'http://{ip}:{port}/ISAPI/PTZCtrl/channels/1/absoluteEx'
    #url = f'http://{ip}:{port}/ISAPI/PTZCtrl/channels/1/capabilities'
    #url = f'http://{ip}:{port}/ISAPI/PTZCtrl/channels/1/RailwayRobot/capabilities?format=json'
    #url = f'http://{ip}:{port}/ISAPI/PTZCtrl/channels/1/absoluteEx/capabilities'

    print(url)
    try:
        ptz = requests.get(url, auth=HTTPDigestAuth(admin,passwd))
        text =ptz.text

        print("text:",text)

        P = text[text.index('<azimuth>')+ len('<azimuth>'):text.index('</azimuth>')]
        T = text[text.index('<elevation>') + len('<elevation>'):text.index('</elevation>')]
        Z = text[text.index('<absoluteZoom>')+ len('<absoluteZoom>'):text.index('</absoluteZoom>')]

        res=[float(P),float(T),float(Z)]
    except Exception as e:
        print("\n无法获取ptz位置信息，报错：{0}".format(str(e)))
    return res

def change_zoom(zoom, ip= "192.168.1.64",port =80,admin=  'admin',passwd= 'fh123456DH'):
    """
    摄像头变焦
    :param zoom:倍数 大于0放大 小于0缩小
    :param second:持续时间
    :return:
    """
    print("zoom:", zoom)
    try:
        session = requests.Session()
        #/ ISAPI / PTZCtrl / channels / < channelID > / absoluteEx
        url = f'http://{ip}/ISAPI/PTZCtrl/channels/1/absoluteEx'
        #print("put  url:", url)
        #param = '<PTZAbsoluteEx><absoluteZoom>%d</absoluteZoom></PTZAbsoluteEx>' % zoom
        param="""<?xml version="1.0" encoding="UTF-8"?>
                <PTZAbsoluteEx version="2.0" xmlns="http://www.isapi.org/ver20/XMLSchema">
	                <elevation>-5.450</elevation>
	                <azimuth>2.900</azimuth>
	                <absoluteZoom>%f</absoluteZoom>
	                <focus>44050</focus>
	                <focalLen>4188</focalLen>
	                <horizontalSpeed>10.00</horizontalSpeed>
	                <verticalSpeed>10.00</verticalSpeed>
	                <zoomType>absoluteZoom</zoomType>  
                    <objectDistance>3.0</objectDistance>
                </PTZAbsoluteEx>""" % zoom
        
        text = session.put(url, data=param,auth=HTTPDigestAuth(admin, passwd))
        #print("param text:", param)
        #print("return text:", text)
        #time.sleep(5)
    except Exception as e:
        print("\n无法设置ptz位置信息，报错：{0}".format(str(e)))


if __name__ == "__main__":

    # --------- 1.获取ptz元组的值 ----------
    ip = "192.168.1.64"
    port=80
    admin = 'admin'
    passwd = 'fh123456DH'
    current_ptz = get_ptz(ip=ip,port=port,admin=admin, passwd = passwd)
    get_ptz(ip=ip, port=port, admin=admin, passwd=passwd)
    change_zoom(2.0, ip, port, admin, passwd)

    #change_zoom_2(1.5,ip=ip,port=port,admin=admin, passwd = passwd)


