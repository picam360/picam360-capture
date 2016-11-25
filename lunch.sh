#!/bin/bash

CURRENT=$(cd $(dirname $0) && pwd)
cd $CURRENT

usage_exit() {
        echo "Usage: $0 [-w width] [-h height] item ..." 1>&2
        exit 1
}

CAM_NUM=1
CAM_WIDTH=1024
CAM_HEIGHT=1024
BITRATE=8000000
RENDER_WIDTH=1440
RENDER_HEIGHT=720
BACKGROUND=false
REMOTE=false
STEREO=
MODE=
DIRECT=
FPS=30

while getopts n:w:h:W:H:BsCEFf:rD OPT
do
    case $OPT in
        n)  CAM_NUM=$OPTARG
            ;;
        w)  CAM_WIDTH=$OPTARG
            ;;
        h)  CAM_HEIGHT=$OPTARG
            ;;
        W)  RENDER_WIDTH=$OPTARG
            ;;
        H)  RENDER_HEIGHT=$OPTARG
            ;;
        B)  BACKGROUND=true
            ;;
        s)  STEREO="-s"
            ;;
        C)  MODE="-C"
            ;;
        E)  MODE="-E"
            ;;
        F)  MODE="-F"
            ;;
        f)  FPS=$OPTARG
            ;;
        r)  REMOTE=true
            ;;
        D)  DIRECT="-D"
            ;;
        \?) usage_exit
            ;;
    esac
done

if [ -e cam0 ]; then
	rm cam0
fi
mkfifo cam0

if [ -e cam1 ]; then
	rm cam1
fi
mkfifo cam1

if [ -e cmd ]; then
	rm cmd
fi
mkfifo cmd

if [ $REMOTE = true ]; then
	socat -u udp-recv:9000 - > cam0 & socat -u udp-recv:9001 - > cam1 &
elif [ $DIRECT = "" ]; then
	raspivid -n -t 0 -w $CAM_WIDTH -h $CAM_HEIGHT -ih -b $BITRATE -fps $FPS -o - > cam0 &
fi

if [ $BACKGROUND = true ]; then
	./picam360-capture.bin -n $CAM_NUM -w $CAM_WIDTH -h $CAM_HEIGHT -W $RENDER_WIDTH -H $RENDER_HEIGHT $DIRECT $MODE $STEREO < cmd &
else
	./picam360-capture.bin -n $CAM_NUM -w $CAM_WIDTH -h $CAM_HEIGHT -W $RENDER_WIDTH -H $RENDER_HEIGHT $DIRECT $MODE $STEREO -p
fi