#!/bin/bash
#
# start only mavros node

source ros_environment_f3.sh
# connect to local port 14551 which voxl-vision-px4 uses for local comms with mavros
#roslaunch mavros px4.launch fcu_url:=udp://127.0.0.1:14551@:14551 tgt_system:=${PX4_SYS_ID}


roslaunch mavros px4_modified.launch fcu_url:=udp://127.0.0.1:14551@:14551 tgt_system:=${PX4_SYS_ID}

## this line allows you to optionally let mavros handle the communication to QGC.
## note this will result in redundant communication with QGC if voxl-vision-px4
## manually or automatically connects to QGC.
# roslaunch mavros px4.launch fcu_url:=udp://127.0.0.1:14551@:14551 gcs_url:=udp://@${QGC_IP} tgt_system:=${PX4_SYS_ID}
