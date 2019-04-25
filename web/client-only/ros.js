import getConfig from 'next/config'
import roslib from 'roslib'

const {publicRuntimeConfig} = getConfig();

var ros = undefined
if(typeof window !== 'undefined') {
  let server = `ws://${window.location.hostname}:${publicRuntimeConfig.port_rosbridge}`
  ros = new roslib.Ros({url: server})

  ros.on('connection', function() {
    console.log('roslib: Connected to websocket server.');
  })

  ros.on('error', function(error) {
    console.log('roslib: Error connecting to websocket server: ', error);
  })

  ros.on('close', function() {
    console.log('roslib: Connection to websocket server closed.');
  })
}

export default ros
