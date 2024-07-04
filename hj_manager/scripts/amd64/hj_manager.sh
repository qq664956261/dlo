#!/usr/bin/env bash
node_path="../../../../"
echo $#
if [ $# == 1 ]; then
  
  node_path=$1
  echo ${node_path}
fi
mkdir /tmp/log
export ROS_LOG_DIR=/tmp/log
export HJ_LOG_CONFIG_PATH=${node_path}/src/hj_manager/config/amd64/hj_log.config
export LOG_RECORDER_NEW_PATH=${node_path}/src/hj_manager/config/amd64/log_recorder.json
export LD_LIBRARY_PATH=${node_path}/src/hj_interface/platforms/amd64:${LD_LIBRARY_PATH}
while true; do
    C0UNT=$(ps -A | grep roscore | wc -l)
    if [ "$C0UNT" != "0" ]; then
        echo "get roscore"
        break
    fi
    roscore &
    echo "need sleep"
    sleep 1
done

PID=`ps -ef | grep -E "collect_node|middleware_node|planning_node|slam_node|utils_node|log_recorder" | grep -v grep | awk '{print $2}'`
for item  in $PID
do
	kill -s 9 $item
	echo "kill -s -9 $item"
done

#rosparam set /hj_log_config_path "${node_path}/src/hj_manager/config/hj_log.config"
./log_recorder > /dev/null &

${node_path}devel/lib/collect_node/collect_node > /dev/null &
${node_path}devel/lib/middleware_node/middleware_node > /dev/null &
${node_path}devel/lib/planning_node/planning_node > /dev/null &
${node_path}devel/lib/slam_node/slam_node > /dev/null &
${node_path}devel/lib/utils_node/utils_node > /dev/null &
