import React, { Component } from 'react'
import { Row } from 'reactstrap'

import getConfig from 'next/config'
const {publicRuntimeConfig} = getConfig();

class TopicDisplayVideo extends Component {
  render() {
    return (
      <Row className='m-auto'>
        <span className='font-weight-bold'>{this.props.topic}</span>
        <img width='100%' src={`http://${this.state.server}/stream?topic=${this.props.topic}&type=ros_compressed`} />
      </Row>
    )
  }

  constructor(props) {
    super(props)
    this.state = {}
  }

  componentDidMount() {
    if(typeof window !== 'undefined' && window.document && window.document.createElement) {
      this.setState({server: `${window.location.hostname}:${publicRuntimeConfig.port_video}`})
    }
  }
}

export default TopicDisplayVideo
