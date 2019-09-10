#!/usr/bin/env python

import rospy
import rospkg
import psutil
import os

rospy.init_node('web_control')
rospack = rospkg.RosPack()

web_env = os.environ.copy()
web_env['ACTOR_ESTOP_TOPIC'] = str(rospy.get_param('~estop_state', '/estop/state'))
web_env['ACTOR_ESTOP_STOP'] = str(rospy.get_param('~estop_stop', '/estop/stop'))
web_env['ACTOR_ESTOP_RESUME'] = str(rospy.get_param('~estop_resume', '/estop/resume'))
web_env['PORT_ROSBRIDGE'] = str(rospy.get_param('~rosbridge_port', '8090'))
web_env['PORT_VIDEO'] = str(rospy.get_param('~video_port', '8091'))
web_env['VIDEO1'] = str(rospy.get_param('~video1'))
web_env['VIDEO2'] = str(rospy.get_param('~video2'))
web_env['VIDEO3'] = str(rospy.get_param('~video3'))
web_env['VIDEO4'] = str(rospy.get_param('~video4'))
web_env['ROS_NS'] = str(rospy.get_namespace())
web_cwd = rospack.get_path('ltu_actor_core') + '/web'
if 'ACTOR_CORE_WEB_HOTBUILD' in web_env.keys():
    web_server = psutil.Popen('npm run dev', shell=True, cwd=web_cwd, env=web_env)
else:
    web_server = psutil.Popen('npm run start', shell=True, cwd=web_cwd, env=web_env)

def on_shutdown():
    web_server.kill()
    if not web_server.wait(timeout=2):
        web_server.terminate()


rospy.on_shutdown(on_shutdown)
rospy.loginfo('LTU-Actor web control!')
rospy.spin()
