R"=====(--[[
  heartbeat()                           toggle the heartbeat status topic to show the route is running
  spin_once()                           allow ROS topics/services to be serviced
  spin_for(int ms)                      delay for milliseconds while also servicing ROS
  send(float linear, float angular)     continuously send cmd commands (forward/reverse & steering)
  send_topic(string topic)              continuously forward a Twist topic as for cmd instead of constants
  last_Float32(string topic)            get the value of a topic (replace Float32 with any std_msgs type)
  pub_Float32(string topic, float val)  oppisite of above, publish on a topic
  estop()                               trigger an estop
]]--


while true
do
  send(0.0, 0.0)
  heartbeat()
  spin_for(500)
end

)====="
