#!/bin/bash

if [ $# -ne 0 ]; then
    echo "Usage: $0"
    exit 1
fi

filename="$(date +%Y.%m%d.%H%M)"
echo ${filename}
mkdir ${filename}
cd ${filename}
pwd
ssm-logger -l config.log -n GM_Config -i 0 &
ssm-logger -l rtk_gnss.log -n RTK-GNSS-F9P -i 0 &
ssm-logger -l imu.log -n imu_fs -i 0 &
ssm-logger -l joystick.log -n joystick -i 0 &
ssm-logger -l OMcntl.log -n OMcntl -i 0 &
ssm-logger -l urg.log -n urg_fs -i 0 &
ssm-logger -l obp.log -n obp_fs -i 0 &
ssm-logger -l wp.log -n wp_gl -i 0 &
ssm-logger -l control.log -n control -i 0 &
ssm-logger -l localizer.log -n localizer -i 0
#bash
