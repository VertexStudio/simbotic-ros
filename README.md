[![simbotic ros docker container](docs/media/simbotic_ros_launch.gif)](https://www.youtube.com/watch?v=E8aY146W6-U)

# Setup Instructions

## Install Docker, Docker Compose and nvidia-docker2
- https://docs.docker.com/compose/install/
- https://docs.docker.com/install/linux/docker-ce/ubuntu/
- https://github.com/NVIDIA/nvidia-docker

## Setup this repository
Clone repository:
```
git clone --recursive git@github.com:Simbotic/simbotic-ros.git
```
Create `.env` file:
```
./setup.sh
```
# Run GPU based container
```
docker-compose -f docker-compose.gpu.yml up
```
# Run CPU based container
```
docker-compose -f docker-compose.cpu.yml up
```
# Demos

## Zero-latency streaming
[![streaming zero-latency from UE4 to ROS](docs/media/streaming_camera_ros_gst.gif)](https://youtu.be/71B5teyduqU)

[![streaming highres from UE4 to ROS](docs/media/streaming_image_ros.gif)](https://youtu.be/71B5teyduqU)

## ROS topics
[![ros topics](docs/media/simbotic_ros_topics.gif)](https://youtu.be/7ecwkY0zGpE)

> Check [GScam docs](https://github.com/ros-drivers/gscam) to configure a camera feed or streaming.