#!/bin/bash
NUMP=`ps -a | grep -o "run.sh" | wc -l`

while [ $NUMP -ge 1 ]
do
	PID=`ps -a | grep -m 1 "run.sh"| awk '{print $1}'`
	kill -9 $PID
	NUMP=`ps -a | grep -o "run.sh" | wc -l`
done
