# LTU-Actor Core

This is the main package of the LTU-Actor autonomous vehicle platfor. It has
the other packages in this organizations as submodules to track the version
used with this version of Core.

## Getting Started

### Install a PPA for a more up to date version of node.js

Ubuntu 16.04 (ROS Kinetic) only ships with version 4.x, and we need
significantly newer.

```sh
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
```

### Installing

First clone this repository into a new workspace

```sh
cd /path/to/my/empty/folder
mkdir src
catkin_make
git clone --recursive git@github.com:LTU-Actor/Core.git ./src/Core
```

Then run [rosdep](http://wiki.ros.org/rosdep) to install all dependencies

```sh
rosdep install --from-paths src --ignore-src -r -y
```

And compile with catkin

```sh
catkin_make
```

### Running

This is not final.

Launch the rosbridge, video server, empty router, and simple webpage with:

```sh
roslaunch ltu_actor_core core.launch
```

then view the page <http://localhost:3000/>
