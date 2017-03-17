// cam0
/usr/bin/raspivid -cd MJPEG -t 0 -co 20 -w 2048 -h 2048 -fps 5 -cs 0 -b 8000000 -o - | /usr/bin/socat - UDP-DATAGRAM:192.168.4.2:9100 &
#/usr/bin/raspivid -ih -t 0 -ex sports -w 1440 -h 1440 -fps 30 -cs 0 -b 2000000 -o - | /usr/bin/socat - UDP-DATAGRAM:192.168.4.2:9100 &
#/usr/bin/raspivid -cd MJPEG -t 0 -ex sports -w 2048 -h 2048 -fps 5 -cs 0 -b 8000000 -o - | nc -l 9100 &

// cam1
/usr/bin/raspivid -cd MJPEG -t 0 -co 20 -w 2048 -h 2048 -fps 5 -cs 1 -b 8000000 -o - | /usr/bin/socat - UDP-DATAGRAM:192.168.4.2:9101 &

//wait for i2c available
sleep 3

// driver
socat -u udp-recv:9001 - | /home/pi/picam360/picam360-driver/picam360-driver.bin | /usr/bin/socat - UDP-DATAGRAM:192.168.4.2:9000 &