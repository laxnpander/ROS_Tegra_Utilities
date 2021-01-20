# ROS_Tegra_Utilities

This repository contains some utility tools for ROS on Jetson TX2 boards. Right now this is limited to a custom jpeg compression node.

## Hardware accelerated jpeg compression

When using the Jetson TX2 for computer vision applications with ROS, typically some driver node is executed using a nodelet. However, these nodelets utilize CPU
implementations of jpeg compression, which tends to slow down the compressed image topic to 1/3 or worse of the raw image rate. When recording datasets or
transferring the images via network the high bandwidth of raw images is often not feasible, but the weak performance of the compression is also a problem.

To improve the performance one can utilize the dedicated onboard chip on Tegra boards for jpeg compression. This accelerates the compression roughly by a factor
of 4 depending on the desired quality.

### Usage

```
roslaunch ros_tegra_utilities jpeg_compression.launch
```

To customize parameters have a look inside the launch file.

| Parameter        | Description           | Default  |
| ------------- |:-------------:| -----:|
| fps | Frame rate of the camera | 15.0 |
| quality | Quality of the compression      | 100 |
| topic_in | Raw camera image input topic | /camera/image_raw |
| topic_out | Compressed image output topic | /tegra_utils/compressed |
