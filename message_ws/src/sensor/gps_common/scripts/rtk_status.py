#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键查看 RTK 状态：从 /nmea_sentence 抓一条 GGA 并解读。

用法（ROS 环境下）:
    python3 rtk_status.py

退出码: 0=RTK固定(正常)  1=有差分未固定  2=单点(差分没到)  3=无NMEA(链路断)
"""
import rospy
import sys
from std_msgs.msg import String

QUALITY = {'0': '无效', '1': '单点定位', '2': 'DGPS伪距差分',
           '4': 'RTK固定解', '5': 'RTK浮点解', '6': '估算'}


def main():
    rospy.init_node('rtk_status', anonymous=True)
    got = {'gga': None}

    def cb(m):
        if got['gga'] is None and 'GGA' in m.data:
            got['gga'] = m.data

    rospy.Subscriber('/nmea_sentence', String, cb)
    t0 = rospy.Time.now()
    while got['gga'] is None and (rospy.Time.now() - t0).to_sec() < 10 \
            and not rospy.is_shutdown():
        rospy.sleep(0.2)

    if got['gga'] is None:
        print('FAIL: 10秒内无 NMEA —— 串口/接收机供电/USB 链路断了')
        sys.exit(3)

    f = got['gga'].split(',')
    q = f[6] if len(f) > 6 else '?'
    diff = f[13] if len(f) > 13 else ''
    print('GGA: %s' % got['gga'].strip())
    print('解状态  : quality=%s (%s)' % (q, QUALITY.get(q, '?')))
    print('卫星/HDOP: %s 颗 / %s' % (f[7] if len(f) > 7 else '?',
                                     f[8] if len(f) > 8 else '?'))
    if diff:
        print('差分龄期: %s s（差分数据已到达）' % diff)
    else:
        print('差分龄期: 无 —— 没收到任何差分数据')

    code = {'4': 0, '5': 1, '2': 1, '1': 2}.get(q, 3)
    verdict = {0: 'RTK 固定解，厘米级，正常可用',
               1: '有差分但未固定：链路不稳或基准站缺载波改正(RTCM3 MSM)',
               2: '单点定位：差分没到，精度米级，不能用于导航',
               3: '无效解'}[code]
    print('结论   : %s' % verdict)
    sys.exit(code)


if __name__ == '__main__':
    main()
