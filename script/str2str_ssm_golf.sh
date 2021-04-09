#! /bin/sh

cd /home/hasegawa/RTKLIB/app/str2str/gcc
sudo ./str2str -in ntrip://l5mt20i5:vz384t@ntrip.ales-corp.co.jp/RTCM32MSM4 -b 1 -out serial://ttyUSB1:460800:n:1:off -c /home/hasegawa/RTKLIB/data/ubx_m8t_bds_raw_5hz.cmd

