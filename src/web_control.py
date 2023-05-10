#!/usr/bin/env python3

import rospy
import rospkg
import psutil
import os
import sys

rospy.init_node('web_control')
rospack = rospkg.RosPack()

web_env = os.environ.copy()
web_env['ACTOR_GPS_POSITION_TOPIC'] = str(rospy.get_param('~gps_position', '/baseline/rover/piksi/position_receiver_0/ros/navsatfix'))
web_env['ACTOR_GPS_NED_TOPIC'] = str(rospy.get_param('~gps_ned', '/baseline/rover/piksi/position_receiver_0/ros/vel_ned'))
web_env['ACTOR_GPS_HEADING_TOPIC'] = str(rospy.get_param('~gps_heading', '/baseline/rover/piksi/position_receiver_0/ros/vel_ned'))
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
#web_env['NPM_CONFIG_PREFIX'] = '/some/path/to/node_modules'
web_cwd = rospack.get_path('ltu_actor_core') + '/web'

rospy.init_node('web_control')

if 'ACTOR_CORE_WEB_HOTBUILD' in web_env.keys():
    web_server = psutil.Popen('npm run dev', shell=True, cwd=web_cwd, env=web_env)
else:
    web_install = psutil.Popen('npm install', shell=True, cwd=web_cwd, env=web_env)
    if web_install.wait(timeout=60) is None:
        web_install.terminate()
        rospy.logfatal('Failed to finish npm install')

    web_build = psutil.Popen('npm run build', shell=True, cwd=web_cwd, env=web_env)
    if web_build.wait(timeout=60) is None:
        web_build.terminate()
        rospy.logfatal('Failed to finish npm run build')

    web_server = psutil.Popen('npm run start', shell=True, cwd=web_cwd, env=web_env)

def on_shutdown():
    web_server.kill()
    if not web_server.wait(timeout=2):
        web_server.terminate()


rospy.on_shutdown(on_shutdown)
rospy.loginfo('LTU-Actor web control!')
rospy.spin()
