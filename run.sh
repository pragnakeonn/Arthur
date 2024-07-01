#!/bin/bash



docker run -it \
    --name="test_interface_docker" \
    --rm --privileged \
    --net=host \
    --env=NVIDIA_VISIBLE_DEVICES=all \
    --env=NVIDIA_DRIVER_CAPABILITIES=all \
    --env=DISPLAY \
    --env=QT_X11_NO_MITSHM=1 \
    -e NVIDIA_VISIBLE_DEVICES=0 \
    -p 11311:11311 \
    test_interface_docker