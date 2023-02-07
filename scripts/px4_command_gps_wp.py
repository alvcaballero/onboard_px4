#!/usr/bin/env python
import rospy

from aerialcore_common.srv import ConfigMission
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix

def send_mission_srv(wp_list):
    rospy.wait_for_service('mission/new')
    try:
        send_mission = rospy.ServiceProxy('mission/new', ConfigMission)
        res = send_mission(waypoint = wp_list)
        return res.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    wp_list = []

    wp1 = NavSatFix()
    wp1.latitude = 38.140263733714214
    wp1.longitude = -3.16965533487803
    wp1.altitude = 60
    wp_list.append(wp1)

    wp2 = NavSatFix()
    wp2.latitude = 38.141423073546
    wp2.longitude = -3.1659135660374886
    wp2.altitude = 70
    wp_list.append(wp2)

    wp3 = NavSatFix()
    wp3.latitude = 38.144233783002754
    wp3.longitude = -3.1661714462168525
    wp3.altitude = 80
    wp_list.append(wp3)

    wp4 = NavSatFix()
    wp4.latitude = 38.14418401
    wp4.longitude = -3.17233927
    wp4.altitude = 60
    wp_list.append(wp4)

    wp5 = NavSatFix()
    wp5.latitude = 38.1436441
    wp5.longitude = -3.17815218
    wp5.altitude = 60
    wp_list.append(wp5)

    wp6 = NavSatFix()
    wp6.latitude = 38.142329520376876
    wp6.longitude = -3.183987481763296
    wp6.altitude = 40
    wp_list.append(wp6)

    wp7 = NavSatFix()
    wp7.latitude = 38.13866147481027
    wp7.longitude = -3.1848903850834915
    wp7.altitude = 35
    wp_list.append(wp7)

    wp8 = NavSatFix()
    wp8.latitude = 38.13772242434103
    wp8.longitude = -3.1800326151547154
    wp8.altitude = 25
    wp_list.append(wp8)

    wp9 = NavSatFix()
    wp9.latitude = 38.13916621
    wp9.longitude = -3.17401076
    wp9.altitude = 25
    wp_list.append(wp9)

    print wp_list

    send_mission_srv(wp_list)

    print "Mission sent"
