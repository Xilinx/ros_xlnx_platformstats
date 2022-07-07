# ROS2 Platformstats Wrapper
[![License](https://img.shields.io/badge/license-MIT-green)](./LICENSE)

ROS2 wrapper for the [platformstats](https://github.com/Xilinx/platformstats) utility which prints stats via diagnostic messages.

## Install
- Install ROS2 as described in [ROS2 Documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)


## Build Instructions

Install build tools:
```
sudo apt install python3-colcon-common-extensions
```

### Install dependencies
Install platformstats:  
See [platformstats](https://github.com/Xilinx/platformstats) README  
Install RQt:
```
sudo apt install ros-humble-rqt*
```

Source the ROS environment:
```
source /opt/ros/humble/setup.sh
```

Clone and build the application:
```
mkdir -p workspace/src
cd workspace
git clone

# Build
colcon build --packages-select ros_platformstats

# Add the built application to current environment
source install/local_setup.sh

# In future, use install/setup.sh to source ROS + app
```
## Run Instructions

1. Source ROS environment e.g `source /opt/ros/humble/setup.sh`
2. Run Publisher in the background: `ros2 run ros_platformstats publisher &`
3. Open Runtime Monitor: `ros2 run rqt_runtime_monitor rqt_runtime_monitor`

![rqt_runtime_monitor](.github/rqt_runtime_monitor.gif)

4. (Alternate) The stats can also be viewed in the terminal by running the
subscriber node: `ros2 run ros_platformstats subscriber`

![subscriber_output](.github/subscriber_output.gif)

## License

```
Copyright (C) 2022 Xilinx, Inc.  All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```
