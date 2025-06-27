#!/bin/bash

script_path=$(dirname $(realpath $0))
echo "build script path: $script_path"

colcon build --symlink-install --cmake-clean-first --cmake-clean-cache