#!/bin/bash
SHARED_DIR=${1:-$(pwd)}
VERSION=${VERSION:-`cat version`}

docker run -it \
    --name dtu_sim2real \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=/tmp/.Xauthority \
    --privileged \
    --network host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $SHARED_DIR:/home/ws \
    -v /dev:/dev \
    -v $XAUTHORITY:/tmp/.Xauthority:ro \
    --shm-size=512M \
    --gpus all \
    esquivelrs/sim2real_omnidrones:$VERSION bash