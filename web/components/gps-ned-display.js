import React, { Component } from 'react'
import { Container, Row, Col } from 'reactstrap'
import ros from 'clientOnly/ros.js'
import roslib from 'roslib'

class TopicGpsNed extends Component {
  render() {
    const xTitle = 'north:'
    const yTitle = 'east:'
    const zTitle = 'down:'
    return (
        <Container>
        <Row xs="3">
            <Col xs="2">
                <span>{this.props.name}</span>
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
        that.setState(
          {
            x: String(vector.x.toFixed(3)),
            y: String(vector.y.toFixed(3)),
            z: String(vector.z.toFixed(3)),
          })
      })
  }

  componentWillUnmount() {
    this.sub.unsubscribe()
    this.sub = null
  }
}

export default TopicGpsNed
