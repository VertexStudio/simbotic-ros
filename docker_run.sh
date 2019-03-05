#!/usr/bin/env bash
if [ "$1" == "gpu" ]; then
    docker run --rm --runtime=nvidia -ti -v $(pwd):/sim --network=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --cap-add=SYS_PTRACE simbotic-cortex/gpu
else
    docker run --rm -ti -v $(pwd):/sim --network=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --cap-add=SYS_PTRACE simbotic-cortex/cpu
fi
