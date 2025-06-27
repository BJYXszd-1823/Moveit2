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
mkdir -p $TARGET_LOG
ros2 launch $TARGET demo.launch.py -d -a 2>&1 | tee ${TARGET_LOG}/$(date +%Y%m%d%H%M%S).log

popd  >>  /dev/null