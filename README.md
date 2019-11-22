# ESP32-CAM-MJPEG-Stream-Decoder-and-Control-Library

![alt text](img url ?raw=true)

## Firmware

## Easy-to-use

### Dependence
- Opencv 3 or 4
- libcurl 7

- wxWidgets 2.8.12(for wxESP32-CAM example)

### Use
Include "ESP32-CAM MJPEG Library" folder into your project. </br>
- ESP32-CAM Library.h
- ESP32-CAM Library.cpp

```cpp
#include "./ESP32-CAM MJPEG Library/ESP32-CAM Library.h"

ESP32_CAM *esp32_cam = new ESP32_CAM(std::string( "192.168.1.254" ));   //ESP32-CAM local IP address

esp32_cam->SetResolution(FRAMESIZE_SVGA); // Set mjpeg video stream resolution

esp32_cam->StartVideoStream();   // Start mjpeg video stream thread

cv::Mat frame = esp32_cam->GetFrame(); // Get mjpeg frame
if(!frame.empty()){
   cv::imshow("Example",frame);  //Show mjpeg video stream
}

```
   

### Command POST and ESP32-CAM GER

In cmd_handler, "var" is variable name, "val" is value of variable, curl POST command is:

```cpp
int ESP32_CAM::SendStream(CURL *curl_object)
{
   long return_code;
   CURLcode response;

   curl_easy_setopt(curl_object,CURLOPT_URL,"192.168.1.254/control?var=framesize&val=7");

   response = curl_easy_perform(curl_object);

   response = curl_easy_getinfo(curl_object, CURLINFO_RESPONSE_CODE, &return_code);

   return (int)return_code;
}
```

"192.168.1.254/control?var=framesize&val=8", "192.168.1.254" is ESP32-CAM IP Address setting from:

```cpp
// Set your Static IP address
IPAddress local_IP(192, 168, 1, 254);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);
WiFi.begin(ssid, password);  
WiFi.config(local_IP, gateway, subnet);
```

In this example, "/control?var=framesize&val=8" POST /control API and set parameter "framesize" value is "8"(FRAMESIZE_SVGA 800x600).

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

Licensing
=======
Copyright (C) 2019  TonyGUO <https://github.com/GCY>.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
