import React, { Component } from 'react';
import Link from 'next/link'
import TopicDisplay from 'components/topic-display.js'

class EStopDisplay extends Component {
  render() {
    return (
      <TopicDisplay name='eStop State' topic='/estop/state' type='std_msgs/Bool' />
    )
  }
}

export default EStopDisplay
