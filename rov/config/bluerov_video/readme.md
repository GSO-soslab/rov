# save video from bluerov camera

- [reference](https://discuss.bluerobotics.com/t/get-video-in-videolan/1884)
- add this to `http://192.168.2.2:2770/camera`
    > ! multiudpsink clients=192.168.2.1:5600,192.168.2.1:2000
- open VLC with this:
    > c=IN IP4 127.0.0.1
    > m=video 2000 RTP/AVP 96
    > a=rtpmap:96 H264/90000
- record video in VLC
- reduce latency: Tools → Preferences → Show Settings -> All → Input Codecs → Advanced → Network Caching (ms)
 `Change the value to 200.`