#!/usr/bin/env bash

echo "USER_ID=$(id -u "${USER}")" > .env
echo "GROUP_ID=$(id -g "${USER}")" >> .env
echo "HOME_SIM=/home/sim" >> .env
echo "CURRENT_DIRECTORY=$(pwd)" >> .env