#!/bin/bash

# Get base.sh funcs
source "$(dirname "$0")/base.sh"

stop_docker

run_docker ncs_ros:latest "/root/app.sh"