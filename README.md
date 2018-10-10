build
```
colon build
```

run rtsp
```bash
ros2 run ros2_videostreamer rtsp_node 
```

or run usb
```bash
ros2 run ros2_videostreamer usb_node 
```

use this command to detect whether rtsp have been published
```bash
ros2 run intra_process_demo image_view_node
```
It will take a lot of time to show the picture, need to figue out why.
