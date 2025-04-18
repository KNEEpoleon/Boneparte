BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)
MODIFIED_BRANCH_NAME=$(echo "$BRANCH_NAME" | sed 's/[^a-zA-Z0-9_\-]/-/g')
MODIFIED_BRANCH_NAME="ament_test"
CONTAINER_NAME="boneparu_${MODIFIED_BRANCH_NAME}"

# MODIFIED_BRANCH_NAME="lrs2tst"


echo $BRANCH_NAME

echo $MODIFIED_BRANCH_NAME

sudo docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v $(pwd)/src:/ros_ws/src --network host $(for device in /dev/video*; do echo --device=$device:$device; done) --gpus all boneparu:${MODIFIED_BRANCH_NAME}

# sudo docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v $(pwd)/src:/ros_ws/src --network host $(for device in /dev/video*; do echo --device=$device:$device; done) --gpus all boneparte:${MODIFIED_BRANCH_NAME}


# export MODIFIED_BRANCH_NAME="fvd_encore"
# sudo docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v $(pwd)/src:/ros_ws/src --network host $(for device in /dev/video*; do echo --device=$device:$device; done) --gpus all paradockerimage:${MODIFIED_BRANCH_NAME}
