#!/bin/sh

command="ps -a
"
SERVER="172.20.10.12"
WIFI_MEC="172.30.22.190"
L5G_MEC="192.168.150.51"
COUNTAINER="172.18.0.4"
THK001_WIFI="172.30.22.191"
THK002_WIFI="172.30.22.192"
THK001_L5G="192.168.200.23"
THK002_L5G="192.168.200.28"
USER="ncos"
NCOSPASS="ncos1234"
SEEDPASS="seed"
#ROSLAUNCH="roslaunch task_programmer nanki.launch"
ROSLAUNCH="ps -a"
#ROSCORE="roscore"
ROSCORE="ls -a"
TEST="cp kuro.txt kuro1.txt"
WIFI_BASH="cp kuro.txt kuro1.txt"
L5G_BASH="cp kuro.txt kuro1.txt"

expect -c "
spawn ssh ${USER}@${SERVER}
set timeout 5
expect \"${USER}@${SERVER}'s password:\"
send \"${NCOSPASS}\n\"
expect \"$\"
send \"source .bashrc\n\"
expect \"$\"
send \"${ROSLAUNCH}\n\"
expect \"$\"
send \"${TEST}\n\"
expect \"$\"

exit 0
"
