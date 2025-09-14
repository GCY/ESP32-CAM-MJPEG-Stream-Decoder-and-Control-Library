#ifndef __ESP32_CAM__
#define __ESP32_CAM__

#include <iostream>
#include <cstdio>
#include <string>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <curl/curl.h>

#ifdef _MAC_
#include <unistd.h>
#include <pthread.h>
#endif

#ifdef _WIN10_
#include <windows.h>
#endif

/**/
const char VIDEO_STREAM_INTERLEAVE[] = "--WINBONDBOUDARY\r\nContent-Type: image/jpeg\r\n\r\n";
const int VIDEO_STREAM_INTERLEAVE_SIZE = 46;

const int VIDEO_STREAM_SERVICE_WAIT_TIME = 30;
const int VIDEO_STREAM_STOP_WAIT_TIME = 1000;

const int COMMUNICATION_TIMEOUT = 180;

const int VIDEO_STREAM_BUFFER_MAX_SIZE = 512000; //500k max.
const int VIDEO_STREAM_BUFFER_MIN_SIZE = 1024; //1k min. QQVGA 160x120 ≈ 3Kb.

/**/
const char JPEG_SOI_MARKER_FIRST_BYTE = 0xFF;
const char JPEG_SOI_MARKER_SECOND_BYTE = 0xD8;

#ifdef _MAC_
void* VideoStreamThreadPasser(void*);
#define strcpy_s(dest,len,src) (strcpy(dest,src))
#endif
#ifdef _WIN10_
DWORD VideoStreamThreadPasser(LPVOID);
void* memmem(const void*, size_t, const void*, size_t);
#endif
size_t WriteCallback(void*, size_t, size_t, void*);
size_t CURLWriteMemoryVideoFrameCallback(void*, size_t, size_t, void*);
void* CURLRealloc(void*, size_t);

typedef struct
{
    char* memory;
    size_t size;
    size_t frame_size;
#ifdef _MAC_
    pthread_mutex_t completed;
#endif
#ifdef _WIN10_
    HANDLE completed;
#endif
    cv::Mat image;
    char* buffer;
    size_t buffer_size;
}VideoStreamMemory;

class ESP32_CAM
{
public:
    ESP32_CAM(const std::string) noexcept;
    ~ESP32_CAM() noexcept;

    ESP32_CAM(const ESP32_CAM&) = delete;
    ESP32_CAM(const ESP32_CAM&&) = delete;
    ESP32_CAM& operator=(const ESP32_CAM&) = delete;
    ESP32_CAM& operator=(const ESP32_CAM&&) = delete;


    void StartVideoStream() noexcept;


    void StopVideoStream() noexcept;

    void SetResolution(int) noexcept;

    void FlashControl(bool) noexcept;


    void VideoStreamThread() noexcept;


    void ClearFrame() noexcept;


    int SendStream(CURL*, std::string) noexcept;

    size_t GetFrameSize() noexcept { return frame.frame_size; }

    std::string GetRSSI() noexcept;

    cv::Mat GetFrame() noexcept;

    ESP32_CAM& operator >> (CV_OUT cv::Mat&) noexcept;


private:
#ifdef _MAC_
    pthread_t video_stream_handle;
#endif
#ifdef _WIN10_
    HANDLE video_stream_handle;
#endif

    VideoStreamMemory frame;
    CURL* curl_video;
    CURL* curl_command;
    CURL* curl_rssi;
    const std::string base_url;
};

extern ESP32_CAM* operator >> (ESP32_CAM*, CV_OUT cv::Mat&);

#endif