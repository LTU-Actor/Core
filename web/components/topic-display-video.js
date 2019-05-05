import React, { Component } from 'react'

import getConfig from 'next/config'
const {publicRuntimeConfig} = getConfig();

class TopicDisplayVideo extends Component {
  render() {
    return (
      <div>
        <span className='font-weight-bold'>{this.props.topic}</span>
        <img src={`http://${this.state.server}/stream?topic=${this.props.topic}&type=ros_compressed`} />
      </div>
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
