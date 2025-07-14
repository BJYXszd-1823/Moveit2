#!/bin/bash

script_path=$(dirname $(realpath $0))
echo "build script path: $script_path"

function usage()
{
    echo "Usage:  [options]"
    echo "Options:"
    echo "  -h, --help                  Show this help message"
    echo "  -c, --clean                 Clean build"
    echo "  -u, --update                Update deps"
}

if  [ $# -eq 0 ];then
    usage
    exit 1
fi

TARGET="*"

while [ $# -gt 0 ]; do
    case "$1" in
        -h | --help)
            usage
            exit 0
            ;;
        -c | --clean)
            echo "clean build"
            rm -rf build install log
            shift 1
            ;;
        -u | --update)
            echo "update deps"
            sudo apt update
            sudo rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
            shift 1
            ;;
        -t | --target)
            echo "build target: $2"
            TARGET="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

colcon build --paths src/$TARGET --symlink-install --cmake-clean-first --cmake-clean-cache