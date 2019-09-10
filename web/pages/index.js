import TopicDisplay from 'components/topic-display.js'
import TopicDisplayVideo from 'components/topic-display-video.js'
import RouteSelect from 'components/route-select.js'
import EstopControl from 'components/estop-control.js'
import getConfig from 'next/config'
import Link from 'next/link'
import {Col, Container, Row} from 'reactstrap'

const {publicRuntimeConfig} = getConfig();
const ros_ns = publicRuntimeConfig.ros_ns;
const video1 = publicRuntimeConfig.video1;
const video2 = publicRuntimeConfig.video2;
const video3 = publicRuntimeConfig.video3;
const video4 = publicRuntimeConfig.video4;

export default () => (
  <Container>
    <Row>
      <Col>
        <Row>
          <Col xs="3"><TopicDisplayVideo topic={`${video1}`} /></Col>
          <Col xs="3"><TopicDisplayVideo topic={`${video2}`} /></Col>
          <Col xs="3"><TopicDisplayVideo topic={`${video3}`} /></Col>
          <Col xs="3"><TopicDisplayVideo topic={`${video4}`} /></Col>
        </Row>

        <TopicDisplay name="Route HB:" topic={`${ros_ns}router/heartbeat`} type="std_msgs/Bool" />
        <TopicDisplay name="Route Index:" topic={`${ros_ns}router/index`} type="std_msgs/Int16" />
        <TopicDisplay name="Route Distance:" topic={`${ros_ns}router/distance`} type="std_msgs/Float32" />
        <TopicDisplay name="estop state manual" topic={publicRuntimeConfig.estop_topic} type="std_msgs/Bool" />
        <RouteSelect />
        <EstopControl />
      </Col>
    </Row>
  </Container>
)
