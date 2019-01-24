import Link from 'next/link'
import TopicDisplay from 'components/topic-display.js'
import EStopDisplay fomr 'components/estop-state.js'

import {Container, Row, Col} from 'reactstrap'

export default () => (
  <Container>
    <Row>
      <Col>
        <TopicDisplay name="Topic from router" topic="/router" type="std_msgs/Int64" />
        <EStopDisplay />
      </Col>
    </Row>
  </Container>
)
