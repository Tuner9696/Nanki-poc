#!/bin/sh

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
ROSLAUNCH="roslaunch task_programmer nanki.launch"
#ROSCORE="roscore"
TEST="ps -a"
WIFI_BASH="cp .bashrc_wifi .bashrc"
L5G_BASH="cp .bashrc_l5G .bashrc"

expect -c "
spawn ssh ${USER}@${WIFI_MEC}
expect \"${USER}@${WIFI_MEC}'s password:\"
send \"${NCOSPASS}\n\"
expect \"$\"
send \"source .bashrc\n\"
expect \"$\"
send \"ssh ${USER}@${COUNTAINER}\n\"
expect \"${USER}@${COUNTAINER}'s password:\"
send \"${NCOSPASS}\n\"
expect \"$\"
send \"source .bashrc\n\"
expect \"$\"
send \"ssh ${SEEDPASS}@${THK002_L5G}\n\"
expect \"${SEEDPASS}@${THK002_L5G}'s password:\"
send \"${SEEDPASS}\n\"
expect \"$\"
send \"${L5G_BASH}\n\"
expect \"$\"
send \"source .bashrc\n\"
expect \"$\"
send \"${ROSLAUNCH}\n\"
interact
"