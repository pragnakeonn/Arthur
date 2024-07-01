#!/bin/bash

xhost +local:root

docker run -it \
    --name="create3_humble_container" \
    --rm --privileged \
    --gpus all \
    --net=host \
    --env=NVIDIA_VISIBLE_DEVICES=all \
    --env=NVIDIA_DRIVER_CAPABILITIES=all \
    --env=DISPLAY \
    --env=QT_X11_NO_MITSHM=1 \
    --runtime=nvidia \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -p 11311:11311 \
    -v /etc/localtime:/etc/localtime:ro \
    -v /home/pragna/ARTHUR/MRS/system_intallation/create3-docker/create3-humble/create3_sim_bckp/:/root/colcon_ws/src/create3_sim/ \
    -v /home/pragna/ARTHUR/MRS/system_intallation/create3-docker/create3-humble/nav2_rl_api/:/root/colcon_ws/src/nav2_rl_api/ \
    create3-humble