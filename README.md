## Stereo Fisheye Rectification
ROS Node for Stereo Image Rectification using the Fisheye Camera Model from
OpenCV

## Usage:

### Calibration Format
An example calibration parameter file (.yml) can be found in
```
/stereo_fisheye_rectify/examples/fisheye_stereo_rectify.yml
```

### Input Format A
This repository supports input data where the stereo images are in a single,
vertically concatenated input image, such as on the Open Vision Computer.
```
roslaunch stereo_fisheye_rectify stereo_fisheye_rectify.launch
```

### Input Format B
Regular stereo image pair input is also supported, using
*message_filters::ApproximateTime* for Time Synchronization between the stereo
image frames
```
roslaunch stereo_fisheye_rectify stereo_fisheye_rectify_split.launch
```


