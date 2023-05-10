import React, { Component } from 'react'
import Link from 'next/link'
import ros from 'clientOnly/ros.js'
import roslib from 'roslib'

class TopicDisplay extends Component {
  render() {
    console.log(this.props)
    return (
      <div>
        <span className="font-weight-bold">{this.props.name}</span>
        &nbsp; &nbsp;
        <span className="font-italic">{this.state.data}</span>
      </div>
    )
  }

  constructor(props) {
    super(props)

    this.state = {
      data: "Not Captured"
    }
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
      that.setState({data: String(msg.data)})
    })
  }

  componentWillUnmount() {
    this.sub.unsubscribe()
    this.sub = null
  }
}

export default TopicDisplay
