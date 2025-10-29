#!/bin/bash

xhost +

# Specify the container name or ID
# CONTAINER_NAME="boneparu_main"
BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)
# MODIFIED_BRANCH_NAME=$(echo "$BRANCH_NAME" | sed 's/[^a-zA-Z0-9_\-]/-/g')
MODIFIED_BRANCH_NAME="main"
IMAGE_NAME="boneparu:${MODIFIED_BRANCH_NAME}"
CONTAINER_NAME="boneparu_${MODIFIED_BRANCH_NAME}"

# Check if the container is running
if [ $(docker ps -q -f name=^/${CONTAINER_NAME}$) ]; then
    echo "Entering the bash shell of the container: $CONTAINER_NAME"
    docker exec -it $CONTAINER_NAME /bin/bash
else
    echo "Container '$CONTAINER_NAME' is not running."
fi

xhost -local:root