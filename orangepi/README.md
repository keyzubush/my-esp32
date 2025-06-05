ffmpeg -f v4l2 -framerate 30 -video_size 640x480 -i /dev/video0 -c:v libx264 -preset ultrafast -tune zerolatency -x264-params nal-hrd=cbr -g 15 -b:v 2M -minrate 2M -maxrate 2M -bufsize 100K -f mpegts tcp://0.0.0.0:1234?listen

ffplay -fflags nobuffer -flags low_delay -framedrop -strict experimental tcp://192.168.1.145:1234

