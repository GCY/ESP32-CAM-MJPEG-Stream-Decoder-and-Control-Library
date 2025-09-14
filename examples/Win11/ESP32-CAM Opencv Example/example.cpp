#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <curl/curl.h>

#include "ESP32-CAM Library.h"

typedef enum {
    FRAMESIZE_96X96,    // 96x96
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_128X128,    // 128x128
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_HQVGA,    // 240x176
    FRAMESIZE_240X240,  // 240x240
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_320X320,  // 320x320
    FRAMESIZE_CIF,      // 400x296
    FRAMESIZE_HVGA,     // 480x320
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_XGA,      // 1024x768
    FRAMESIZE_HD,       // 1280x720
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200
    FRAMESIZE_INVALID
} framesize_t;

int main(int argc, char** argv) {


    ESP32_CAM* esp32_cam = new ESP32_CAM(std::string("192.168.110.48"));

    esp32_cam->StartVideoStream();

    esp32_cam->SetResolution(FRAMESIZE_SVGA);

    while (1) {
        //cv::Mat frame = esp32_cam->GetFrame();
        cv::Mat frame;
        esp32_cam >> frame;
        if (!frame.empty()) {
            std::cout << esp32_cam->GetFrameSize() / 1024.0f << "kb" << std::endl;
            cv::imshow("Example", frame);
        }

        if (cv::waitKey(30) == 27) { // If 'esc' key is pressed, break loop.
            esp32_cam->StopVideoStream();
            break;
        }

    }
    /*
       cv::VideoCapture cap("http://192.168.110.48:81/stream");
       cv::Mat frame;
       while (cap.read(frame)) {
       cv::imshow("ESP32", frame);
       if (cv::waitKey(30) == 27) break; // ESC to quit
       }
       */
}
