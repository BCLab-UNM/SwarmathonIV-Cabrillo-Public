##!#/bin/bash
#Carter's script for building robot testing edition v.0.0.2
#TODO: add lovelace's IP address, test script, use rsync, report status of file transpher
logfile=/tmp/RoboLog`date +20%y.%m.%d.%H.%M.%S`.txt
echo Logging stated sent to $logfile |tee logfile

#need a path for catkin?
catkin build && (echo "Build Successful `date`" |tee $logfile; notify-send "Build Successful `date`") || (echo "Build Failed `date`"|tee $logfile; notify-send "Build Failed `date`"; exit)

robot=$(yad --title="Choose a Robot to deploy to" --list --editable --width=200 --height=150 --column="Name" --column="IP" "lovelace" "1" "maverick" "192.168.1.134" "element618" "192.168.1.139")
echo "user selected $robot" |tee $logfile
robotName=`echo $robot | cut -d"|" -f1`
robotIP=`echo $robot | cut -d"|" -f2`

ping -q -c 2 `echo $robotIP | cut -d"|" -f2` -s 1 &>/dev/null && (echo "Robot is responsive"|tee $logfile; notify-send "Robot is responsive") || (echo "unable to find robot on network at $robotIP"|tee $logfile; notify-send "unable to find robot on network at $robotIP"; exit) 

echo "Transferring built files to $robotName"|tee $logfile; notify-send "Transferring built files to $robotName"
scp -r ../devel robot@$robotIP:~/rover_workspace/devel 
#future change to something like rsync -aux ../devel robot@$robotIP:~/rover_workspace/devel
