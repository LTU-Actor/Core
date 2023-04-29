import React, { Component } from 'react'
import { Container, Row, Col } from 'reactstrap'
import ros from 'clientOnly/ros.js'
import roslib from 'roslib'

class TopicGpsPosition extends Component {
  render() 
  {
    const latitudeTitle = 'lat:'
    const longitudeTitle = 'lon:'
    const altitudeTitle = 'alt:'
    return (
      <Container>
        <Row xs="3">
          <Col xs="2">
            <h5 className='font-weight-bold'>{this.props.name}</h5>
          </Col>
          <Col xs="2">
              <div>
                  <span className="font-weight-bold">{latitudeTitle}</span>
                  &nbsp; &nbsp;
                  <span className="font-italic">{this.state.lat}</span>
              </div>
          </Col>
          <Col xs="2">
              <div>
                  <span className="font-weight-bold">{longitudeTitle}</span>
                  &nbsp; &nbsp;
                  <span className="font-italic">{this.state.long}</span>
              </div>
          </Col>
          <Col xs="2">
              <div>
                  <span className="font-weight-bold">{altitudeTitle}</span>
                  &nbsp; &nbsp;
                  <span className="font-italic">{this.state.alt}</span>
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
      that.setState(
        {
          lat: String(msg.latitude),
          long: String(msg.longitude),
          alt: String(msg.altitude)
        })
    })
  }

  componentWillUnmount() {
    this.sub.unsubscribe()
    this.sub = null
  }
}

export default TopicGpsPosition
