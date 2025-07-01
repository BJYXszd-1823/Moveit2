#!/bin/bash

script_path=$(dirname $(realpath $0))
echo "running $script_path"
RUN_LOG_PATH="rlogs"

function usage()
{
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -h, --help                  Show this help message"
    echo "  -t, --target <target name>  Target to run"
}

if  [ $# -eq 0 ];then
    usage
    exit 1
fi

while [ $# -gt 0 ]; do
    case "$1" in
        -h | --help)
            usage
            exit 0
            ;;
        -t | --target)
            if  [ -z "$2" ]; then
                echo "Error: missing argument for option[-t, --target] "
                usage
                exit 1
            fi
            TARGET=$2
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

pushd $script_path >>  /dev/null

if [ ! -f install/setup.bash ];then
    bash build.sh
fi

source install/setup.bash

ros2 pkg list | grep $TARGET
if [ $? -ne 0 ];then
    echo "package $TARGET not exist"
    exit 1
fi

TARGET_LOG=$RUN_LOG_PATH/$TARGET
mkdir -p $TARGET_LOG/history
mv $TARGET_LOG/*.log $TARGET_LOG/history

case $TARGET in
    "miniarm")
        killall gzserver
        killall gzclient
        ros2 launch $TARGET gazebo.launch.py -d -a 2>&1 | tee ${TARGET_LOG}/$(date +%Y%m%d%H%M%S).log
        ;;
    "moveit_arm")
        killall ros2
        killall gzserver
        killall gzclient
        killall rviz2
        ros2 launch $TARGET gazebo.launch.py -d 2>&1 | tee ${TARGET_LOG}/gazebo_$(date +%Y%m%d%H%M%S).log
        # sleep 5
        # ros2 launch $TARGET my_moveit_rviz.launch.py -d -a 2>&1 | tee ${TARGET_LOG}/rviz_$(date +%Y%m%d%H%M%S).log &
        ;;
    "rm_serial_driver")
        ros2 launch $TARGET serial_driver.launch.py -d -a 2>&1 | tee ${TARGET_LOG}/$(date +%Y%m%d%H%M%S).log
        ;;
    *)
        ros2 launch $TARGET demo.launch.py -d -a 2>&1 | tee ${TARGET_LOG}/$(date +%Y%m%d%H%M%S).log
        ;;
esac


popd  >>  /dev/null