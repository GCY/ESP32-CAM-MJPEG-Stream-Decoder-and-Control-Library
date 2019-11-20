#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <curl/curl.h>

#include <unistd.h>
#include <pthread.h>

#include "ESP32-CAM Library.h"

typedef enum {
   FRAMESIZE_QQVGA,    // 160x120
   FRAMESIZE_QQVGA2,   // 128x160
   FRAMESIZE_QCIF,     // 176x144
   FRAMESIZE_HQVGA,    // 240x176
   FRAMESIZE_QVGA,     // 320x240
   FRAMESIZE_CIF,      // 400x296
   FRAMESIZE_VGA,      // 640x480
   FRAMESIZE_SVGA,     // 800x600
   FRAMESIZE_XGA,      // 1024x768
   FRAMESIZE_SXGA,     // 1280x1024
   FRAMESIZE_UXGA,     // 1600x1200
   FRAMESIZE_QXGA,     // 2048*1536
   FRAMESIZE_INVALID
} framesize_t;

int main(int argc,char**argv){


   ESP32_CAM *esp32_cam = new ESP32_CAM(std::string("192.168.110.254"));

   esp32_cam->StartVideoStream();

   esp32_cam->SetResolution(FRAMESIZE_SVGA);

   while (1) {
      cv::Mat frame = esp32_cam->GetFrame();
      if(!frame.empty()){
	 std::cout << esp32_cam->GetFrameSize()/1024.0f << "kb" << std::endl;
	 cv::imshow("Example",frame);
      }
      if (cv::waitKey(30) == 27) // If 'esc' key is pressed, break loop.
      {
	 esp32_cam->StopVideoStream();
	 break;
      }
   }
}
