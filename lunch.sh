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
CODEC=H264
STREAM=
AUTO_CALIBRATION=
VIEW_COODINATE=manual
DEBUG=false
KBPS=
FRAME0_PARAM=

while getopts ac:n:w:h:psf:Dv:g0: OPT
do
    case $OPT in
        a)  AUTO_CALIBRATION="-a"
            ;;
        c)  CODEC=$OPTARG
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
        D)  DIRECT="-D"
            ;;
        v)  VIEW_COODINATE=$OPTARG
            ;;
        g)  DEBUG=true
            ;;
        0)  FRAME0_PARAM=$OPTARG
            ;;
        \?) usage_exit
            ;;
    esac
done

if [ -e cam0 ]; then
	rm cam0
fi
mkfifo cam0
chmod 0666 cam0

if [ -e cam1 ]; then
	rm cam1
fi
mkfifo cam1
chmod 0666 cam1

if [ $DIRECT = ]; then
	sudo killall raspivid
	if [ $CODEC = "MJPEG" ]; then
#		raspivid -cd MJPEG -n -t 0 -w $CAM_WIDTH -h $CAM_HEIGHT -ex sports -b $BITRATE -fps $FPS -o - > cam0 &
		raspivid -cd MJPEG -n -t 0 -w $CAM_WIDTH -h $CAM_HEIGHT -b $BITRATE -fps $FPS -o - > cam0 &
	else
		raspivid -n -t 0 -w $CAM_WIDTH -h $CAM_HEIGHT -ex sports -ih -b $BITRATE -fps $FPS -o - > cam0 &
	fi
fi

#picam360-capture

sudo killall picam360-capture.bin
if [ $DEBUG = "true" ]; then

echo b main > gdbcmd
echo r $AUTO_CALIBRATION -c $CODEC -n $CAM_NUM -w $CAM_WIDTH -h $CAM_HEIGHT $DIRECT $STEREO $PREVIEW -v $VIEW_COODINATE -F \"$FRAME0_PARAM\" >> gdbcmd
gdb ./picam360-capture.bin -x gdbcmd

else

./picam360-capture.bin $AUTO_CALIBRATION -c $CODEC -n $CAM_NUM -w $CAM_WIDTH -h $CAM_HEIGHT $DIRECT $STEREO $PREVIEW -v $VIEW_COODINATE -F "$FRAME0_PARAM"

fi