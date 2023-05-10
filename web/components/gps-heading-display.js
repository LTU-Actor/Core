import React, { Component } from 'react'
import { Container, Row, Col } from 'reactstrap'
import ros from 'clientOnly/ros.js'
import roslib from 'roslib'

class TopicGpsHeading extends Component {
  render() {
    // TODO: Head to this
    // const compassTitle = 'Compass:'
    // const compassValue = compassStringFromAngle(this.props.headingAngle)
    // const headingTitle = 'Heading:'
    // const headingValue = this.props.headingAngle

    // Start here
    console.log(this.state)
    const xTitle = 'x:'
    const yTitle = 'y:'
    const zTitle = 'z:'
    const wTitle = 'w:'
    return (
        <Container>
        <Row xs="3">
            <Col xs="2">
                <h5 className='font-weight-bold'>{this.props.name}</h5>
            </Col>
            <Col xs="2">
                <div>
                    <span className="font-weight-bold">{xTitle}</span>
                    &nbsp; &nbsp;
                    <span className="font-italic">{this.state.x}</span>
                </div>
            </Col>
            <Col xs="2">
                <div>
                    <span className="font-weight-bold">{yTitle}</span>
                    &nbsp; &nbsp;
                    <span className="font-italic">{this.state.y}</span>
                </div>
            </Col>
            <Col xs="2">
                <div>
                    <span className="font-weight-bold">{zTitle}</span>
                    &nbsp; &nbsp;
                    <span className="font-italic">{this.state.z}</span>
                </div>
            </Col>
            {/* <Col xs="2">
                <div>
                    <span className="font-weight-bold">{wTitle}</span>
                    &nbsp; &nbsp;
                    <span className="font-italic">{this.state.w}</span>
                </div>
            </Col> */}
        </Row>
        </Container>
    )
  }

  constructor(props) {
    super(props)
    this.state = {}
  }

  componentDidMount() {
    var that = this
    this.sub = new roslib.Topic({
      ros: ros,
      name: that.props.topic,
      messageType: that.props.type
    })

    var that = this
    this.sub.subscribe(function(msg) {
        console.log(msg)
        console.log(msg.vector)
        var vector = msg.vector
        console.log(vector)
        console.log(vector.x)
        console.log(vector.y)
        console.log(vector.z)
        // headingAngle = headingAngleFromNedVector(vector)
        that.setState(
          {
            x: String(vector.x),
            y: String(vector.y),
            z: String(vector.z),
            // w: String(poseOrientation.w),
            // headingAngle: headingAngle
          })
      })
  }

  componentWillUnmount() {
    this.sub.unsubscribe()
    this.sub = null
  }

  compassStringFromAngle(angle) {
    switch (true) {
        case (angle < 68):
            return "Northeast"
        case (angle < 90):
            return "East"
        case (angle < 113):
            return "Southeast"
        case (angle < 158):
            return "South"
        case (angle < 203):
            return "South"
        case (angle < 248):
            return "Southwest"
        case (angle < 293):
            return "West"
        case (angle < 338):
            return "Northwest"
        default:
            return "North"
    }
  }

  headingAngleFromPoseOrientation(poseOrientation) {
    // TODO: translation poseOrientation to compass heading
    return 0
  }
}

export default TopicGpsHeading
