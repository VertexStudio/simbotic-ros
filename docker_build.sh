#!/usr/bin/env bash
BUILD=cpu
if [ "$1" ]; then
    BUILD=$1
fi
echo "Building $BUILD..."
docker build --build-arg USER_ID="$(id -u "${USER}")" --build-arg GROUP_ID="$(id -g "${USER}")" -f Dockerfile."$BUILD" -t simbotic-ros/"$BUILD" .
