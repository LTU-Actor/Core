import React, { Component } from 'react';
import {Row,Col,Button} from 'reactstrap'
import getConfig from 'next/config'
import ros from 'clientOnly/ros.js'
import roslib from 'roslib'

const {publicRuntimeConfig} = getConfig();
const estop_stop = publicRuntimeConfig.estop_stop
const estop_resume = publicRuntimeConfig.estop_resume


class EstopControl extends Component {
  render() {
    return (
      <div>
        <Row>
          <Button className='col-auto mr-1' color='danger' onClick={this.stop}>eStop</Button>
          <Button className='col-auto mr-1' color='success' onClick={this.resume}>Resume</Button>
        </Row>
      </div>
    )
  }

  constructor(props) {
    super(props)

    this.stop = this.stop.bind(this)
    this.resume = this.resume.bind(this)
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

  componentWillUnmount() {
    this.stopSrv = null
    this.resumeSrv = null
  }
}

export default EstopControl

