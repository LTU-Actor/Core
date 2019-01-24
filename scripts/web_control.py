#!/usr/bin/env python

import rospy

if __name__ == '__main__':
    rospy.init_node('web_control')
    rospy.loginfo('LTU-Actor web control!')
    rospy.spin()
