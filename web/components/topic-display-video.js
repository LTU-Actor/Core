import React, { Component } from 'react'

const port = 8091

class TopicDisplayVideo extends Component {
  render() {
    return (
      <div>
        <span className='font-weight-bold'>{this.props.topic}</span>
        <img src={`http://${this.state.server}/stream?topic=${this.props.topic}`} />
      </div>
    )
  }

  constructor(props) {
    super(props)
    this.state = {}
  }

  componentDidMount() {
    if(typeof window !== 'undefined' && window.document && window.document.createElement) {
      this.setState({server: `${window.location.hostname}:${port}`})
    }
  }
}

export default TopicDisplayVideo
