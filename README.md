# ESP32-CAM-MJPEG-Stream-Decoder-and-Control-Library

![alt text](img url ?raw=true)

## Firmware

## Easy-to-use

### Dependence


- Include "ESP32-CAM MJPEG Library" folder into your project.
   - ESP32-CAM Library.h
   - ESP32-CAM Library.cpp
   
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
