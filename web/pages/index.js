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
        <TopicDisplay name="Topic from router" topic={`${ros_ns}router/heartbeat`} type="std_msgs/Bool" />
        <TopicDisplay name="estop state manual" topic={publicRuntimeConfig.estop_topic} type="std_msgs/Bool" />
        <TopicDisplayVideo topic="/camera/image_color" />
        <RouteSelect />
        <EstopControl />
      </Col>
    </Row>
  </Container>
)
