import TopicDisplay from 'components/topic-display.js'
import TopicDisplayVideo from 'components/topic-display-video.js'
import getConfig from 'next/config'
import Link from 'next/link'
import {Col, Container, Row} from 'reactstrap'

const {publicRuntimeConfig} = getConfig();

export default () => (
  <Container>
    <Row>
      <Col>
        <TopicDisplay name="Topic from router" topic="/router" type="std_msgs/Int64" />
        <TopicDisplay name="estop state manual" topic={publicRuntimeConfig.estop_topic} type="std_msgs/Bool" />
        <TopicDisplayVideo topic="/videotopic" />
      </Col>
    </Row>
  </Container>
)
