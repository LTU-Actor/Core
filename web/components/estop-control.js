import React, { Component } from 'react';
import {Row,Col,Button} from 'reactstrap'
import getConfig from 'next/config'
import ros from 'clientOnly/ros.js'
import roslib from 'roslib'

const {publicRuntimeConfig} = getConfig();
const estop_stop = publicRuntimeConfig.estop_stop
const estop_resume = publicRuntimeConfig.estop_resume
const vehicle_enable = publicRuntimeConfig.vehicle_enable


class EstopControl extends Component {
  render() {
    return (
      <div>
        <Row>
          <Button className='col-auto mr-1' color='danger' onClick={this.stop}>eStop</Button>
          <Button className='col-auto mr-1' color='success' onClick={this.resume}>Resume</Button>
          <Button className='col-auto mr-1' color='primary' onClick={this.enable_vehicle}>Enable DBW</Button>
        </Row>
      </div>
    )
  }

  constructor(props) {
    super(props)

    this.stop = this.stop.bind(this)
    this.resume = this.resume.bind(this)
    this.enable_vehicle = this.enable_vehicle.bind(this)
  }

  componentDidMount() {
    this.stopSrv = new roslib.Service({
      ros: ros,
      name: estop_stop,
      serviceType: 'Empty'
    })

    this.resumeSrv = new roslib.Service({
      ros: ros,
      name: estop_resume,
      serviceType: 'Trigger'
    })

    this.enableVehiclePub = new roslib.Topic({
      ros: ros,
      name: vehicle_enable,
      messageType: 'std_msgs/Empty'
    })
  }

  stop() {
    this.stopSrv.callService(new roslib.ServiceRequest({}), function(res) {})
  }

  resume() {
    this.resumeSrv.callService(new roslib.ServiceRequest({}), function(res) {
      if(!res.success)
        alert("Failed to resume: " + res.message)
    })
  }

  enable_vehicle() {
    var emptyMsg = new roslib.Message({})
    this.enableVehiclePub.publish(emptyMsg)
    console.log(emptyMsg)
  }

  componentWillUnmount() {
    this.stopSrv = null
    this.resumeSrv = null
    this.enableVehiclePub = null
  }
}

export default EstopControl

