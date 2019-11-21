# ESP32-CAM-MJPEG-Stream-Decoder-and-Control-Library

![alt text](img url ?raw=true)

## Firmware

## Easy-to-use

### Dependence
- Opencv3 or 4
- libcurl

### Use
Include "ESP32-CAM MJPEG Library" folder into your project. </br>
- ESP32-CAM Library.h
- ESP32-CAM Library.cpp

```cpp
#include "./ESP32-CAM MJPEG Library/ESP32-CAM Library.h"

ESP32_CAM *esp32_cam = new ESP32_CAM(std::string( "192.168.1.254" ));   //ESP32-CAM local IP address

esp32_cam->StartVideoStream();   // Start mjpeg video stream thread

cv::Mat frame = esp32_cam->GetFrame(); // Get mjpeg frame
if(!frame.empty()){
   cv::imshow("Example",frame);  //Show mjpeg video stream
}

```
   

   
### MJPEG Stream Format

--WINBONDBOUDARY\r\n </br>
Content-Type: image/jpeg\r\n\r\n </br>
JPEG Binary(0xFF,0xD8 ... 0xFF,0xD9) </br>
. </br>
. </br>
. </br>
--WINBONDBOUDARY\r\n </br>
Content-Type: image/jpeg\r\n\r\n </br>
JPEG Binary(0xFF,0xD8 ... 0xFF,0xD9) </br>   

## Examples

## Reference

https://github.com/GCY/wxRovio

https://github.com/tobybreckon/roviolib

https://pjreddie.com/darknet/yolo/

https://github.com/spmallick/learnopencv/tree/master/ObjectDetection-YOLO

https://github.com/CMU-Perceptual-Computing-Lab/openpose

https://github.com/spmallick/learnopencv/tree/master/OpenPose-Multi-Person

https://randomnerdtutorials.com/esp32-cam-video-streaming-face-recognition-arduino-ide/
