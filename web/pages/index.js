import TopicDisplay from 'components/topic-display.js'
import TopicDisplayVideo from 'components/topic-display-video.js'
import RouteSelect from 'components/route-select.js'
import EstopControl from 'components/estop-control.js'
import TopicGpsPosition from 'components/gps-position-display.js'
import TopicGpsNed from 'components/gps-ned-display.js'
import TopicGpsHeading from 'components/gps-heading-display.js'
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
          <Col>
            <Container>
              <Row>{'Blob'}</Row>
              <Row>
                <Col>
                  <Row xs="1">
                    <TopicDisplayVideo topic={`${video1}`} />
                  </Row>
                  <Row xs="1">
                    <TopicDisplayVideo topic={`${video2}`} />
                  </Row>
                </Col>
                <Col>
                  <Row xs="1">
                    <TopicDisplayVideo topic={`${video4}`} />
                  </Row>
                </Col>
              </Row>
            </Container>
          </Col>
          <Col>
            <Container>
            <Row>{'Stop Sign Detect'}</Row>
              <Row>
                <Col>
                  <Row xs="2">
                    <TopicDisplayVideo topic={`${video3}`} />
                  </Row>
                </Col>
              </Row>
            </Container>
          </Col>
        </Row>

        <TopicGpsPosition name="GPS position" topic={publicRuntimeConfig.gps_position} type="sensor_msgs/NavSatFix" />
        {/* <TopicGpsNed name="ned velocity" topic={publicRuntimeConfig.gps_ned} type="geometry_msgs/Vector3Stamped" /> */}
        {/* <TopicGpsHeading name="" topic={publicRuntimeConfig.gps_ned} type="geometry_msgs/Vector3Stamped" /> */}
        
        <TopicDisplay name="Route HB:" topic={`${ros_ns}router/heartbeat`} type="std_msgs/Bool" />
        <TopicDisplay name="Route Index:" topic={`${ros_ns}router/index`} type="std_msgs/Int16" />
        <TopicDisplay name="Route Distance:" topic={`${ros_ns}router/distance`} type="std_msgs/Float32" />
        <TopicDisplay name="estop state manual" topic={publicRuntimeConfig.estop_topic} type="std_msgs/Bool" />
        
        <EstopControl />
        <RouteSelect />
      </Col>
    </Row>
  </Container>
)
