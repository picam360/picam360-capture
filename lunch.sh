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
AUTO_CALIBRATION=
VIEW_COODINATE=MANUAL

while getopts ac:n:w:h:W:H:psCEFf:rDSv: OPT
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
        v)  VIEW_COODINATE=$OPTARG
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

if [ -e status ]; then
	rm status
fi
mkfifo status
chmod 0666 status

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

if [ -e driver ]; then
	rm driver
fi
mkfifo driver
chmod 0666 driver

if [ $REMOTE = true ]; then
	sudo killall socat
	socat PIPE:driver UDP-DATAGRAM:192.168.4.1:9001 &
	socat -u udp-recv:9000 - > status & socat -u udp-recv:9100 - > cam0 & socat -u udp-recv:9101 - > cam1 &
elif [ $DIRECT = ]; then
	sudo killall raspivid
	if [ $CODEC = "MJPEG" ]; then
#		raspivid -cd MJPEG -n -t 0 -w $CAM_WIDTH -h $CAM_HEIGHT -ex sports -b $BITRATE -fps $FPS -o - > cam0 &
		raspivid -cd MJPEG -n -t 0 -w $CAM_WIDTH -h $CAM_HEIGHT -b $BITRATE -fps $FPS -o - > cam0 &
	else
		raspivid -n -t 0 -w $CAM_WIDTH -h $CAM_HEIGHT -ex sports -ih -b $BITRATE -fps $FPS -o - > cam0 &
	fi
fi

sudo killall picam360-capture.bin
./picam360-capture.bin $AUTO_CALIBRATION -c $CODEC -n $CAM_NUM -w $CAM_WIDTH -h $CAM_HEIGHT $DIRECT $STEREO $PREVIEW -F "-W $RENDER_WIDTH -H $RENDER_HEIGHT $MODE $STREAM_PARAM -v $VIEW_COODINATE"
