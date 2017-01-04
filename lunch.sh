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
PREVIEW=
REMOTE=false
STEREO=
MODE=
DIRECT=
FPS=30
CODEC=H264
STREAM=false
STREAM_PARAM=

while getopts c:n:w:h:W:H:psCEFf:rDS OPT
do
    case $OPT in
        c)  CODEC=$OPTARG
            ;;
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
        p)  PREVIEW="-p"
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
        S)  STREAM=true
            ;;
        \?) usage_exit
            ;;
    esac
done

if [ $STREAM = true ]; then
	if [ -e /tmp/stream ]; then
		rm /tmp/stream
	fi
	mkdir /tmp/stream
	chmod 0777 /tmp/stream
	export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
	sudo killall mjpg_streamer 
	mjpg_streamer -i "input_file.so -f /tmp/stream" &
	STREAM_PARAM="-o /tmp/stream/steam.jpeg"
fi


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

if [ -e cmd ]; then
	rm cmd
fi
mkfifo cmd
chmod 0666 cmd

if [ $REMOTE = true ]; then
	socat -u udp-recv:9000 - > cam0 & socat -u udp-recv:9001 - > cam1 &
elif [ $DIRECT = ]; then
	if [ $CODEC = "MJPEG" ]; then
#		raspivid -cd MJPEG -n -t 0 -w $CAM_WIDTH -h $CAM_HEIGHT -ex sports -b $BITRATE -fps $FPS -o - > cam0 &
		raspivid -cd MJPEG -n -t 0 -w $CAM_WIDTH -h $CAM_HEIGHT -b $BITRATE -fps $FPS -o - > cam0 &
	else
		raspivid -n -t 0 -w $CAM_WIDTH -h $CAM_HEIGHT -ex sports -ih -b $BITRATE -fps $FPS -o - > cam0 &
	fi
fi

./picam360-capture.bin -c $CODEC -n $CAM_NUM -w $CAM_WIDTH -h $CAM_HEIGHT -W $RENDER_WIDTH -H $RENDER_HEIGHT $DIRECT $MODE $STEREO $STREAM_PARAM $PREVIEW
