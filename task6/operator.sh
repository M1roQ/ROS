#!/bin/bash

main="../operator.sh"

. "$main"

export DOCKERFILE="../../Dockerfile"
export IMAGE="m1roq/my-ros-jazzy:ex00"
export NAME="ros-jazzy-task6"
export SRC="../task6"
export DST="/home/valerie/workbench"

main_func "$1"
