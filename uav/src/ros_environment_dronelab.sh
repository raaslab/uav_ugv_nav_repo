
#!/bin/bash                                                                                                                                                                                                        
                                                                                                                                                                                                               

# load main ros environment                                                                                                                                                                                        
if [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
elif [ -f /opt/ros/kinetic/setup.bash ]; then
    source /opt/ros/kinetic/setup.bash
elif [ -f /opt/ros/indigo/setup.bash ]; then
    source /opt/ros/indigo/setup.bash
fi

source catkin_ws/install/setup.bash
unset ROS_HOSTNAME                                                                                                                                                                                                 

# configure ROS IPs here                                                                                                                                                                                           
export ROS_MASTER_IP=192.168.78.165                                                                                                                                                                                   
export ROS_IP=192.168.78.165                                                                                                                                                                                 
export ROS_MASTER_URI=http://${ROS_MASTER_IP}:11311/                                                                                                                                                               
# mavros needs to know what PX4's system id is                                                                                                                                                                     
export PX4_SYS_ID=1  
~                    