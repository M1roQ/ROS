#!/bin/bash

main="../operator.sh"

. "$main"

export DOCKERFILE="../../Dockerfile"
export IMAGE="m1roq/my-ros-jazzy:ex00"
export NAME="ros-jazzy-ex01"
export SRC="../task3"
export DST="/home/valerie/workbench"

main_func "$1"
