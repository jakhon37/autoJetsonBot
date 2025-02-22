
docker build -t autonomous_ros_project .


docker run -d \
  --privileged \
  --restart=always \
  --name auto_ros4 \
  --network=host \
  autonomous_ros_project3
