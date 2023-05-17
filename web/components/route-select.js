import React, { Component } from 'react';
import {Row,Col,Button} from 'reactstrap'
import getConfig from 'next/config'
import ros from 'clientOnly/ros.js'
import roslib from 'roslib'

const {publicRuntimeConfig} = getConfig();
const ros_ns = publicRuntimeConfig.ros_ns


class RouteSelect extends Component {
  render() {
    return (
      <div>
        <Row>
          <Button className='col-auto' onClick={this.refreshRoutes}>&#8635;</Button>
          <select className="col border m-1 rounded bg-light" onChange={this.fileNameOnChange} value={this.state.fileName}>
            {this.state.routeFiles.map(function(listValue, i){
              return <option key={i}>{listValue}</option>;
            })}
          </select>
          <Button className='col-auto mr-1' onClick={this.save}>Save</Button>

          {/* commenting out the 'Load' button, as the dropdown loads the necessary file, and it only causes confusion */}
          {/* perhaps, if the file structure where complicated, it would be helpful... or the files had simplier names */}
          {/* <Button className='col-auto mr-1' color='primary' onClick={this.load}>Load</Button> */}

          <Button className='col-auto mr-1' color='success' onClick={this.apply}>Apply</Button>
        </Row>
        <Row>
          <textarea className="col border rounded bg-light mb-1" value={this.state.route} style={{height: "400px", fontFamily: 'monospace'}} onChange={this.routeOnChange} />
        </Row>
      </div>
    )
  }

  constructor(props) {
    super(props)

    this.state = {
      route: "",
      fileName: "",
      routeFiles: []
    }

    this.save = this.save.bind(this)
    this.load = this.load.bind(this)
    this.apply = this.apply.bind(this)
    this.fileNameOnChange = this.fileNameOnChange.bind(this)
    this.routeOnChange = this.routeOnChange.bind(this)
    this.refreshRoutes = this.refreshRoutes.bind(this)
  }

  componentDidMount() {
    this.getCurrentRoute = new roslib.Service({
      ros: ros,
      name: `${ros_ns}router/get_current_route`,
      serviceType: 'GetCurrentRoute'
    })

    this.getRouteList = new roslib.Service({
      ros: ros,
      name: `${ros_ns}router/get_route_list`,
      serviceType: 'GetRouteList'
    })

    this.loadRoute = new roslib.Service({
      ros: ros,
      name: `${ros_ns}router/load_route`,
      serviceType: 'LoadRoute'
    })

    this.saveRoute = new roslib.Service({
      ros: ros,
      name: `${ros_ns}router/save_route`,
      serviceType: 'saveRoute'
    })

    this.setTempRoute = new roslib.Service({
      ros: ros,
      name: `${ros_ns}router/set_temporary_route`,
      serviceType: 'setTemporaryRoute'
    })

    this.refreshRoutes()

    var that = this
    this.getCurrentRoute.callService(new roslib.ServiceRequest({}), function(res) {
      that.setState({fileName: res.filename, route: res.content})
    })
  }

  apply() {
    let route_str = this.state.route
    this.setTempRoute.callService(new roslib.ServiceRequest({
      content: route_str
    }), function(res) {
      if(!res.success)
        alert("Failed to apply")
    })
  }

  load() {
    var that = this
    var keep_going = true

    this.loadRoute.callService(new roslib.ServiceRequest({
      filename: this.state.fileName
    }), function(res) {
      if(!res.success) {
        alert("Failed to load")
        keep_going = false
      }
    })

    if(keep_going)
      this.refreshRoutes()
  }

  save() {
    this.saveRoute.callService(new roslib.ServiceRequest({
      filename: this.state.fileName,
      content: this.state.route
    }), function(res) {
      if(!res.success)
        alert("Failed to save")
    })

    this.apply()
  }

  refreshRoutes() {
    var that = this;

    this.getRouteList.callService(new roslib.ServiceRequest({}), function(res) {
        that.setState({routeFiles: res.routes});
    })

    this.getCurrentRoute.callService(new roslib.ServiceRequest({}), function(res) {
      that.setState({fileName: res.filename, route: res.content})
    })
  }

  fileNameOnChange(e) {
    this.setState({fileName: e.target.value})

    var that = this
    var keep_going = true

    this.loadRoute.callService(new roslib.ServiceRequest({
      filename: e.target.value
    }), function(res) {
      if(!res.success) {
        alert("Failed to load")
        keep_going = false
      }
    })

    if(keep_going)
      this.refreshRoutes()
  }

  routeOnChange(e) {
    this.setState({route: e.target.value})
  }

  componentWillUnmount() {
    this.getCurrentRoute = null
    this.getRouteList = null
    this.loadRoute = null
    this.saveRoute = null
    this.setTempRoute = null
  }
}

export default RouteSelect

