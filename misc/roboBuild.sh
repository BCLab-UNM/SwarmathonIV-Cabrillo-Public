##!#/bin/bash
#Carters Notes on script for building robot script
logfile=RoboLog`date +20%y.%m.%d.%H.%M.%S`.txt
echo Logging to file $logfile
#echo message |tee file

#need a path for catkin?
catkin build |tee $logfile && (echo "Build Successful `date`" |tee $logfile; notify-send "Build Successful `date`") || (echo "Build Failed `date`"|tee $logfile; notify-send "Build Failed `date`")
#if build failed need a way to stop the script nicely

robot=$(yad --title="Choose a Robot to deploy to" --list --editable --width=200 --height=150 --column="Name" --column="IP" "lovelace" "1" "maverick" "192.168.1.134" "element618" "192.168.1.139")
robotName=`echo $robot | cut -d"|" -f1`
robotIP=`echo $robot | cut -d"|" -f2`

ping -q -c 2 `echo $robotIP | cut -d"|" -f2` -s 1 &>/dev/null && (echo "Robot is responsive"|tee $logfile; notify-send "Robot is responsive") || (notify-send "unable to find robot on network at $robotIP") #not working properly

echo "Transphering built files to robot"|tee $logfile; notify-send "Robot is responsive"
scp -r ../devel robot@$robotIP:~/rover_workspace/devel

