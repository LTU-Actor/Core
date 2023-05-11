import React, { Component } from 'react'
import { Container, Row, Col } from 'reactstrap'
import ros from 'clientOnly/ros.js'
import roslib from 'roslib'

class TopicGpsHeading extends Component {
  render() {
    const compassTitle = 'Compass:'
    const headingTitle = 'Heading:'
    return (
        <Container>
        <Row xs="3">
            <Col xs="2">
                <h5 className='font-weight-bold'>{this.props.name}</h5>
            </Col>
            <Col xs="2">
                <div>
                    <span className="font-weight-bold">{compassTitle}</span>
                    &nbsp; &nbsp;
                    <span className="font-italic">{this.state.compassString}</span>
                </div>
            </Col>
            <Col xs="2">
                <div>
                    <span className="font-weight-bold">{headingTitle}</span>
                    &nbsp; &nbsp;
                    <span className="font-italic">{this.state.headingAngle}</span>
                </div>
            </Col>
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
        var vector = msg.vector
        console.log(vector.x)
        console.log(vector.y)
        console.log(vector.z)
        headingAngle = headingAngleFromNedVector(vector)
        compassString = compassStringFromAngle(headingAngle)
        that.setState(
          {
            headingAngle: headingAngle,
            compassString: compassString
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

  headingAngleFromPoseOrientation(nedVector) {
    const north = nedVector.x
    const east = nedVector.y
    
    // AU white paper - section 4.3 ECEF Coordinates and Heading-Pitch-Roll Conversion
    // https://webarchive.nla.gov.au/awa/20060808232012/http://pandora.nla.gov.au/pan/24764/20060809-0000/DSTO-TN-0640.pdf
    var degrees = Math.atan2(east, north)

    // degrees = (degrees + 360.0) % 360.0

    return degrees
  }
}

export default TopicGpsHeading
