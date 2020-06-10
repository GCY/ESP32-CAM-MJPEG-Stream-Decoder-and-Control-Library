#include "ESP32-CAM Library.h"



ESP32_CAM::ESP32_CAM(const std::string base_url) noexcept:base_url(base_url)
{
   curl_command = curl_easy_init();

   curl_easy_setopt(curl_command,CURLOPT_VERBOSE,0);
   curl_easy_setopt(curl_command,CURLOPT_NOPROGRESS,1);
   curl_easy_setopt(curl_command,CURLOPT_TIMEOUT,COMMUNICATION_TIMEOUT);

   curl_rssi = curl_easy_init();

   curl_easy_setopt(curl_rssi,CURLOPT_VERBOSE,0);
   curl_easy_setopt(curl_rssi,CURLOPT_NOPROGRESS,1);
   curl_easy_setopt(curl_rssi,CURLOPT_TIMEOUT,COMMUNICATION_TIMEOUT);   

   curl_video = curl_easy_init();
   curl_easy_setopt(curl_video,CURLOPT_VERBOSE,0);
   curl_easy_setopt(curl_video,CURLOPT_NOPROGRESS,1);
   curl_easy_setopt(curl_video,CURLOPT_TIMEOUT,COMMUNICATION_TIMEOUT);

   frame.memory = nullptr;
   frame.size = 0;
   frame.buffer = nullptr;
   frame.buffer_size = 0;

#ifdef _MAC_
   pthread_mutex_init(&(frame.completed),nullptr);
#endif
#ifdef _WIN10_
   frame.completed = CreateMutex(nullptr,FALSE,nullptr);
#endif
}

ESP32_CAM::~ESP32_CAM() noexcept
{
   curl_easy_cleanup(curl_command);
   curl_easy_cleanup(curl_rssi);
   curl_easy_cleanup(curl_video);
   ClearFrame();
}

void ESP32_CAM::StartVideoStream() noexcept
{
#ifdef _MAC_
   if(pthread_create(&(video_stream_handle),nullptr,VideoStreamThreadPasser,(void*)this)){}
#endif
#ifdef _WIN10_
   video_stream_handle = CreateThread(nullptr,0,(LPTHREAD_START_ROUTINE)VideoStreamThreadPasser,(LPVOID)this,0,nullptr);
#endif
}

void ESP32_CAM::StopVideoStream() noexcept
{
#ifdef _MAC_
   if(pthread_cancel(video_stream_handle)){
      ClearFrame();
   }
#endif
#ifdef _WIN10_
   TerminateThread(video_stream_handle, 0);
   CloseHandle(video_stream_handle);
   ClearFrame();
#endif
}     

void ESP32_CAM::SetResolution(int resolution) noexcept
{
   std::stringstream ss;
   ss << resolution;
   std::string resolution_str = ss.str();   

   std::string command("/control?var=framesize&val=");
   SendStream(curl_command,command + resolution_str);
}

void ESP32_CAM::FlashControl(bool on) noexcept
{
   if(on == true){
      SendStream(curl_command,std::string("/led?var=flash&val=1"));
   }
   else{
      SendStream(curl_command,std::string("/led?var=flash&val=0"));
   }
}

void ESP32_CAM::VideoStreamThread() noexcept
{
   curl_easy_setopt(curl_video,CURLOPT_WRITEFUNCTION,CURLWriteMemoryVideoFrameCallback);
   curl_easy_setopt(curl_video,CURLOPT_WRITEDATA,(void*) &frame);

   if(SendStream(curl_video,std::string(":81/stream"))){
   }
}

void ESP32_CAM::ClearFrame() noexcept
{
   if ((frame.size > 0) && (frame.memory != nullptr)){
      frame.size = 0;
      free(frame.memory);
      frame.memory = nullptr;
   }

   if ((frame.buffer_size > 0) && (frame.buffer != nullptr)){
      frame.buffer_size = 0;
      free(frame.buffer);
      frame.buffer = nullptr;
   }
}

int ESP32_CAM::SendStream(CURL *curl_object,std::string function_url) noexcept
{
   long return_code;
   CURLcode response;

   curl_easy_setopt(curl_object,CURLOPT_URL,(base_url+function_url).c_str());

   response = curl_easy_perform(curl_object);

   response = curl_easy_getinfo(curl_object, CURLINFO_RESPONSE_CODE, &return_code);

   return (int)return_code;
}

std::string ESP32_CAM::GetRSSI() noexcept
{
   std::string buffer;

   curl_easy_setopt(curl_rssi,CURLOPT_WRITEFUNCTION,WriteCallback);
   curl_easy_setopt(curl_rssi,CURLOPT_WRITEDATA,&buffer); 

   if(SendStream(curl_rssi,std::string("/RSSI"))){
   }  

   return buffer;
}

cv::Mat ESP32_CAM::GetFrame() noexcept
{
   cv::Mat return_data;

#ifdef _MAC_
   pthread_mutex_lock( &(frame.completed));
   return_data = frame.image.clone();
   pthread_mutex_unlock( &(frame.completed));
#endif
#ifdef _WIN10_
   WaitForSingleObject(frame.completed, INFINITE);
   return_data = frame.image.clone();
   ReleaseMutex(frame.completed);
#endif
   return return_data;
}

ESP32_CAM& ESP32_CAM::operator >> (CV_OUT cv::Mat &dst) noexcept
{
   dst = GetFrame();
   return *this;   
}




ESP32_CAM* operator >> (ESP32_CAM *esp32_cam,CV_OUT cv::Mat &dst)
{ 
   (*esp32_cam) >> dst;
   return esp32_cam; 
}

#ifdef _MAC_
void* VideoStreamThreadPasser(void* args)
{
   ESP32_CAM *esp32_cam = (ESP32_CAM*)args;
   while(true)
   {
      esp32_cam->VideoStreamThread();
      usleep(VIDEO_STREAM_SERVICE_WAIT_TIME * 1000); // assume ~30 fps max.
   }
   sleep(VIDEO_STREAM_STOP_WAIT_TIME);
   return 0;
}
#endif
#ifdef _WIN10_
DWORD VideoStreamThreadPasser(LPVOID lpParameter)
{
   ESP32_CAM *esp32_cam = (ESP32_CAM*)lpParameter;
   while(true)
   {
      esp32_cam->VideoStreamThread();
      Sleep(VIDEO_STREAM_SERVICE_WAIT_TIME); // assume ~30 fps max.
   }
   Sleep(VIDEO_STREAM_STOP_WAIT_TIME);
   return 0;
}
#endif

#ifdef _WIN10_
void* memmem(const void *buf,size_t buf_len,const void *byte_sequence,size_t byte_sequence_len)
{

   BYTE *bf = (BYTE *)buf;
   BYTE *bs = (BYTE *)byte_sequence;
   BYTE *p  = bf;

   while (byte_sequence_len <= (buf_len - (p - bf))){
      UINT b = *bs & 0xFF;
      if ((p = (BYTE *) memchr(p, b, buf_len - (p - bf))) != nullptr){
	 if ((memcmp(p, byte_sequence, byte_sequence_len)) == 0){
	    return p;
	 }
	 else{
	    p++;
	 }
      }
      else{
	 break;
      }
   }
   return nullptr;
}
#endif

size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

size_t CURLWriteMemoryVideoFrameCallback(void *ptr,size_t size,size_t nmemb,void *data)
{
   if(!ptr){return 0;}

   size_t realsize = size * nmemb;
   VideoStreamMemory *frame = (VideoStreamMemory*)data;

   void *buffer_temp;
   size_t buffer_size_temp;

   if(realsize < VIDEO_STREAM_INTERLEAVE_SIZE){
      frame->buffer = (char*)realloc(frame->buffer, frame->buffer_size + realsize);
      memcpy(&(frame->buffer[frame->buffer_size]),ptr,realsize);
      frame->buffer_size += realsize;

      return realsize;
   }

   if(frame->buffer){
      frame->buffer = (char*)realloc(frame->buffer, frame->buffer_size + realsize);
      memcpy(&(frame->buffer[frame->buffer_size]),ptr,realsize);
      frame->buffer_size += realsize;

      buffer_temp = frame->buffer;
      buffer_size_temp = frame->buffer_size;
   }
   else{
      buffer_temp = ptr;
      buffer_size_temp = realsize;
   }

   char *interleave = (char*)memmem((char*)buffer_temp,buffer_size_temp,VIDEO_STREAM_INTERLEAVE,VIDEO_STREAM_INTERLEAVE_SIZE);

   if((interleave) && (frame->size < VIDEO_STREAM_BUFFER_MAX_SIZE)){
      size_t beforeinterleave = (interleave - (char*)buffer_temp);
      frame->memory = (char*)CURLRealloc(frame->memory,(frame->size) + beforeinterleave + 1);
      if(!frame->memory){
	 frame->size = 0;
	 return 0;
      }

      memcpy(&(frame->memory[frame->size]),buffer_temp,beforeinterleave);
      frame->size +=  beforeinterleave;
      frame->memory[frame->size] = 0;

      char *p_interleave = (char*)memmem((char*)frame->memory,frame->size,VIDEO_STREAM_INTERLEAVE,VIDEO_STREAM_INTERLEAVE_SIZE);

      if(p_interleave){
	 size_t img_size = frame->size - (p_interleave - frame->memory) - VIDEO_STREAM_INTERLEAVE_SIZE;

	 frame->frame_size = img_size;

	 if((img_size >= (VIDEO_STREAM_INTERLEAVE_SIZE + VIDEO_STREAM_BUFFER_MIN_SIZE) &&
		  (p_interleave[VIDEO_STREAM_INTERLEAVE_SIZE] & JPEG_SOI_MARKER_FIRST_BYTE) &&
		  (p_interleave[VIDEO_STREAM_INTERLEAVE_SIZE + 1] & JPEG_SOI_MARKER_SECOND_BYTE))){
#ifdef _MAC_
	    pthread_mutex_lock(&(frame->completed));
#endif
#ifdef _WIN10_
	    WaitForSingleObject(frame->completed, INFINITE);
#endif

	    frame->image = cv::Mat(1, img_size, CV_8UC1, &(p_interleave[VIDEO_STREAM_INTERLEAVE_SIZE]));
	    frame->image = cv::imdecode(frame->image, cv::IMREAD_UNCHANGED/*CV_LOAD_IMAGE_UNCHANGED*/);


#ifdef _MAC_
	    pthread_mutex_unlock(&(frame->completed));
#endif
#ifdef _WIN10_
	    ReleaseMutex(frame->completed);
#endif
	 }
      }

      if(frame->memory){
	 free(frame->memory);
      }
      frame->memory = nullptr;

      size_t frominterleave = buffer_size_temp - beforeinterleave;
      frame->memory = (char*)CURLRealloc(frame->memory,frominterleave + 1);

      if(!frame->memory){
	 frame->size = 0;
	 return 0;
      }

      memcpy(frame->memory,interleave,frominterleave);
      frame->size = frominterleave;
      frame->memory[frame->size] = 0;
   }
   else{
      if(frame->size > VIDEO_STREAM_BUFFER_MAX_SIZE){
	 frame->size = 0;

	 if (frame->memory){
	    free(frame->memory);
	 }
	 frame->memory = nullptr;
      }

      frame->memory = (char*)CURLRealloc(frame->memory,(frame->size) + buffer_size_temp + 1);
      if(!frame->memory){
	 frame->size = 0;
	 return 0;
      }
      memcpy(&(frame->memory[frame->size]), buffer_temp,buffer_size_temp);
      frame->size += buffer_size_temp;
      frame->memory[frame->size] = 0;
   }

   if(frame->buffer){
      frame->buffer_size = 0;
      free(frame->buffer);
      frame->buffer = nullptr;
   }

   return realsize;
}

void* CURLRealloc(void *ptr,size_t size)
{
   if(ptr){
      return realloc(ptr,size);
   }
   else{
      return malloc(size);
   }
}
