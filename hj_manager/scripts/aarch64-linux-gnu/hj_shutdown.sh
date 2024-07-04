#!/usr/bin/env bash
PID=`ps -ef | grep -E "monitor.sh|collect_node|middleware_node|planning_node|slam_node|utils_node|roscore|rosmaster|log_recorder" | grep -v grep | awk '{print $2}'`
for item  in $PID
do
	kill -s 9 $item
	echo "kill -s -9 $item"
done
