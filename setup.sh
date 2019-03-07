#!/usr/bin/env bash

echo "USER_ID=$(id -u "${USER}")" > .env
echo "GROUP_ID=$(id -g "${USER}")" >> .env