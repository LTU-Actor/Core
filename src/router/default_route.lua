R"=====(--[[
  heartbeat()                           toggle the heartbeat status topic to show the route is running
  spin_once()                           allow ROS topics/services to be serviced
  spin_for(int ms)                      delay for milliseconds while also servicing ROS
  send(float linear, float angular)     continuously send cmd commands (forward/reverse & steering)
  send_topic(string topic)              continuously forward a Twist topic as for cmd instead of constants
  dist(double lat, double long)         get the distance in meters from the gps point
  last_float32(string topic)            get the value of a topic (replace Float32 with any std_msgs type)
  pub_float32(string topic, float val)  oppisite of above, publish on a topic
  info_index(int i)                     show an int "index" on the webpage
  info_distance(float d)                show a float "distance" on the wepbage
  vehicle_enable()                      enable the drive-by-wire
  estop()                               trigger an estop
]]--


while true
do
  send(0.0, 0.0)
  heartbeat()
  spin_for(500)
end

)====="
