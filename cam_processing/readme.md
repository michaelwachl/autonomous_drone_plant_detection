# Camera Stream Processing

## Overview

This package can be used as a template for future video stream processing. E.g. to denoise the images. 

## Usage
Start roscore if not already started
```
roscore
```
starte node
```
rosrun cam_processing vcam_processing.py
```

## Subscripted
* **`/camera/image_raw`** Image

## Published
* **`/camera/image_raw_denoised`** Image
