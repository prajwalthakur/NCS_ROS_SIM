#!/bin/bash

docker build --rm  $@ -t ncs_ros:latest -f "$(dirname "$0")/../../docker/ncs_cuda_126.Dockerfile" "$(dirname "$0")/../.."