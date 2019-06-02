import TopicDisplay from 'components/topic-display.js'
import TopicDisplayVideo from 'components/topic-display-video.js'
import RouteSelect from 'components/route-select.js'
import EstopControl from 'components/estop-control.js'
import getConfig from 'next/config'
import Link from 'next/link'
import {Col, Container, Row} from 'reactstrap'

const {publicRuntimeConfig} = getConfig();
const ros_ns = publicRuntimeConfig.ros_ns;

export default () => (
  <Container>
    <Row>
      <Col>
        <TopicDisplayVideo topic="/camera/image_color" />
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
