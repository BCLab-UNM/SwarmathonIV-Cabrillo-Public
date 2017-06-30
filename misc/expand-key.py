#! /usr/bin/python 

import pexpect 
import parted 
import sys
import os
import re

from parted import Constraint
from parted import Disk 

if os.geteuid() != 0 :
    print "You must run this command as root like this:"
    print "  sudo", sys.argv[0]
    exit (-1)

with open('/proc/mounts', 'r') as f :
    mounts = f.readlines()

for mountline in mounts : 
    mount = mountline.split(' ')
    if mount[1] == '/':
        partdev = mount[0]
        break

print "Root is mounted on:", partdev
diskdev = partdev.replace('3', '')
print "Partitioned device is:", diskdev

print "Attempting to get parted to fix the label geometry..."
partfix = pexpect.spawn('/sbin/parted ' + diskdev + ' print')
partfix.logfile = sys.stdout
partfix.sendline('fix')
partfix.sendline('fix')
partfix.expect(pexpect.EOF)

print "Resizing partition..."
device = parted.getDevice(diskdev)
disk = Disk(device)
part = disk.getPartitionByPath(partdev)
cons = Constraint(device=device)
disk.maximizePartition(part, cons)

print "Committing changes to the partition table..."
disk.commit()

print "Calling resize2fs to resize the filesystem..."
resize = pexpect.spawn('/sbin/resize2fs ' + partdev)
resize.logfile = sys.stdout 
partfix.expect(pexpect.EOF, timeout=120)
