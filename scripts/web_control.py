#!/usr/bin/env python

import rospy
import rospkg
import psutil

rospack = rospkg.RosPack()

web_cwd = rospack.get_path('ltu_actor_core') + '/web'
web_server = psutil.Popen('npm run start', shell=True, cwd=web_cwd)


def on_shutdown():
    web_server.kill()
    if not web_server.wait(timeout=2):
        web_server.terminate()


rospy.init_node('web_control')
rospy.on_shutdown(on_shutdown)
rospy.loginfo('LTU-Actor web control!')
rospy.spin()
