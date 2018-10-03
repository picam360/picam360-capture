#!/bin/bash

source ~/.picam360rc

CURRENT=$(cd $(dirname $0) && pwd)
cd $CURRENT

usage_exit() {
        echo "Usage: $0 [-w width] [-h height] item ..." 1>&2
        exit 1
}

CAM_NUM=1
CAM_WIDTH=2048
CAM_HEIGHT=2048
BITRATE=8000000
RENDER_WIDTH=1440
RENDER_HEIGHT=720
PREVIEW=
STEREO=
MODE=
DIRECT=
FPS=30
STREAM=
AUTO_CALIBRATION=
VIEW_COODINATE=manual
MPU_TYPE=manual
DEBUG=false
KBPS=
FRAME0_PARAM=

while getopts an:w:h:psf:dv:M:g0: OPT
do
    case $OPT in
        a)  AUTO_CALIBRATION="-a"
            ;;
        n)  CAM_NUM=$OPTARG
            ;;
        w)  CAM_WIDTH=$OPTARG
            ;;
        h)  CAM_HEIGHT=$OPTARG
            ;;
        p)  PREVIEW="-p"
            ;;
        s)  STEREO="-s"
            ;;
        f)  FPS=$OPTARG
            ;;
        d)  DIRECT="-d"
            ;;
        v)  VIEW_COODINATE=$OPTARG
            ;;
        M)  MPU_TYPE=$OPTARG
            ;;
        g)  DEBUG=true
            ;;
        0)  FRAME0_PARAM=$OPTARG
            ;;
        \?) usage_exit
            ;;
    esac
done

#picam360-capture

sudo killall picam360-capture.bin
if [ $DEBUG = "true" ]; then

echo b main > gdbcmd
echo r $AUTO_CALIBRATION -n $CAM_NUM -w $CAM_WIDTH -h $CAM_HEIGHT $DIRECT $STEREO $PREVIEW -v $VIEW_COODINATE -M MPU_TYPE -F \"$FRAME0_PARAM\" >> gdbcmd
gdb ./picam360-capture.bin -x gdbcmd

else

./picam360-capture.bin $AUTO_CALIBRATION -n $CAM_NUM -w $CAM_WIDTH -h $CAM_HEIGHT $DIRECT $STEREO $PREVIEW -v $VIEW_COODINATE -M MPU_TYPE -F "$FRAME0_PARAM"

fi