docker rm -f s2r2025_client

docker run -id --name s2r2025_client --gpus all \
    --privileged=true \
    --network=host \
    --ipc=host \
    --pid=host \
    -e ROS_DOMAIN_ID=99 \
    -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    -e RCUTILS_COLORIZED_OUTPUT=1 \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    discoverse/s2r2025_client:real_v1 bash

xhost +

