#!/bin/bash

FASTDDS_SERVER_ID=0
FASTDDS_UDP_PORT=11811
FASTDDS_PATTERN="fastdds discovery --server-id=${FASTDDS_SERVER_ID} --udp-port ${FASTDDS_UDP_PORT}"
FASTDDS_CMD=(fastdds discovery --server-id="${FASTDDS_SERVER_ID}" --udp-port "${FASTDDS_UDP_PORT}")


if pgrep -f "${FASTDDS_PATTERN}" >/dev/null; then
    echo "Fast DDS discovery server is already running."
else
    echo "Starting Fast DDS discovery server on UDP port ${FASTDDS_UDP_PORT}."
    "${FASTDDS_CMD[@]}"
fi

export ROS_DISCOVERY_SERVER="10.0.0.20:11811"
. ./install/setup.bash
