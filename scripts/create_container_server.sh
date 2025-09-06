docker rm -f block_challenge_server

docker run -id --name block_challenge_server --gpus all \
    --privileged=true \
    --network=host \
    --ipc=host \
    --pid=host \
    -e ROS_DOMAIN_ID=99 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/input:/dev/input \
    xmartev/block_challenge_server:release_v0 bash

xhost +