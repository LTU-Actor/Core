if(typeof window !== 'undefined') {
  let server = 'ws://' + window.location.hostname + ':8080'
  let roslib = require('roslib')
  let ros = new roslib.Ros({url: 'ws://localhost:8080/'})

  ros.on('connection', function() {
    console.log('roslib: Connected to websocket server.');
  })

  ros.on('error', function(error) {
    console.log('roslib: Error connecting to websocket server: ', error);
  })

  ros.on('close', function() {
    console.log('roslib: Connection to websocket server closed.');
  })

  module.exports = ros
} else {
  module.exports = undefined
}
