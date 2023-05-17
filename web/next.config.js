const withCSS = require('@zeit/next-css')
const {BundleAnalyzerPlugin} = require('webpack-bundle-analyzer')
const {ANALYZE} = process.env
const path = require('path')

module.exports = withCSS({
  webpack: function(config, {isServer}) {
    if (ANALYZE) {
      config.plugins.push(new BundleAnalyzerPlugin(
          {analyzerMode: 'server', analyzerPort: isServer ? 8888 : 8889, openAnalyzer: true}))
    }

    config.resolve.alias.components = path.resolve(__dirname, 'components')
    config.resolve.alias.clientOnly = path.resolve(__dirname, 'client-only')

    // Import these filetypes with file loader
    // Things like LeafLet require this to not throw errors
    config.module.rules.push({test: /\.(gif|svg|jpg|png)$/, use: ['file-loader']})

    return config
  },
  publicRuntimeConfig: {
    gps_position: process.env.ACTOR_GPS_POSITION_TOPIC,
    gps_ned: process.env.ACTOR_GPS_NED_TOPIC,
    estop_topic: process.env.ACTOR_ESTOP_TOPIC,
    estop_stop: process.env.ACTOR_ESTOP_STOP,
    estop_resume: process.env.ACTOR_ESTOP_RESUME,
    vehicle_enable: process.env.ACTOR_DBW_ENABLE_VEHICLE,
    port_video: process.env.PORT_VIDEO,
    port_rosbridge: process.env.PORT_ROSBRIDGE,
    ros_ns: process.env.ROS_NS,
    video1: process.env.VIDEO1,
    video2: process.env.VIDEO2,
    video3: process.env.VIDEO3,
    video4: process.env.VIDEO4
  }
})
