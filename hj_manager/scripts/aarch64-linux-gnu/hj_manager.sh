#!/usr/bin/env bash
ABSOLUTE_PATH="/home/robot/data/hj"

source /opt/ros/melodic/setup.bash
export LD_LIBRARY_PATH=/${ABSOLUTE_PATH}/lib:${LD_LIBRARY_PATH}:/${ABSOLUTE_PATH}/../glog

export HJ_LOG_CONFIG_PATH="${ABSOLUTE_PATH}/config/hj_log.config"
export LOG_RECORDER_NEW_PATH="${ABSOLUTE_PATH}/config/log_recorder.json"
#export HJ_ALL_LOG_CLOSE=close
#export HJ_LOG_CLOSE_collect_node=close
mkdir /tmp/logging
rm /tmp/logging/log_err

while true; do
    COUNT=$(ps -A | grep roscore | wc -l)
    if [ "$COUNT" != "0" ]; then
        break
    fi
    roscore &
    sleep 2 # wait roscore running
done

PID=`ps -ef | grep -E "monitor.sh|collect_node|middleware_node|planning_node|slam_node|utils_node|log_recorder" | grep -v grep | awk '{print $2}'`
for item  in $PID
do
	kill -s 9 $item
	echo "kill -s -9 $item"
done

rosparam set /hj_so_path "${ABSOLUTE_PATH}/lib"
rosparam set /hj_config_path "${ABSOLUTE_PATH}/config"
./log_recorder > /dev/null &

${ABSOLUTE_PATH}/bin/collect_node > /dev/null &
${ABSOLUTE_PATH}/bin/middleware_node > /dev/null &
${ABSOLUTE_PATH}/bin/planning_node > /dev/null &
${ABSOLUTE_PATH}/bin/slam_node > /dev/null &
${ABSOLUTE_PATH}/bin/utils_node > /dev/null &

sleep 2
${ABSOLUTE_PATH}/bin/monitor.sh  &
