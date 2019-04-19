let videoserver = ''
if(typeof window !== 'undefined')
  videoserver = window.location.hostname + ':8081'
else
  videoserver = 'localhost:8081'

const TopicDisplayVideo = (props) => (
  <div>
    <span className='font-weight-bold'>{props.topic}</span>
    <img src={videoserver+'/stream?topic='+props.topic} />
  </div>
)

export default TopicDisplayVideo
