#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <random>

#include <wx/wx.h>
#include <wx/stdpaths.h>


#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "ESP32-CAM Library.h"

struct KeyPoint{
   KeyPoint(cv::Point point,float probability){
      this->id = -1;
      this->point = point;
      this->probability = probability;
   }

   int id;
   cv::Point point;
   float probability;
};
/*
const char keypointsMapping[][255] = {
   "Nose", "Neck",
   "R-Sho", "R-Elb", "R-Wr",
   "L-Sho", "L-Elb", "L-Wr",
   "R-Hip", "R-Knee", "R-Ank",
   "L-Hip", "L-Knee", "L-Ank",
   "R-Eye", "L-Eye", "R-Ear", "L-Ear"
};
*/
struct ValidPair{
   ValidPair(int aId,int bId,float score){
      this->aId = aId;
      this->bId = bId;
      this->score = score;
   }

   int aId;
   int bId;
   float score;
};

const std::vector<std::pair<int,int>> mapIdx = {
   {31,32}, {39,40}, {33,34}, {35,36}, {41,42}, {43,44},
   {19,20}, {21,22}, {23,24}, {25,26}, {27,28}, {29,30},
   {47,48}, {49,50}, {53,54}, {51,52}, {55,56}, {37,38},
   {45,46}
};

const std::vector<std::pair<int,int>> posePairs = {
   {1,2}, {1,5}, {2,3}, {3,4}, {5,6}, {6,7},
   {1,8}, {8,9}, {9,10}, {1,11}, {11,12}, {12,13},
   {1,0}, {0,14}, {14,16}, {0,15}, {15,17}, {2,17},
   {5,16}
};

enum{
   ID_SCREEN = 100
};

enum{
   ID_EXIT = 200,
   ID_START_MJPEG_STREAM,
   ID_STOP_MJPEG_STREAM,
   ID_RESOLUTION,
   ID_VERTICAL_FLIP,
   ID_HORIZONTAL_FLIP,
   ID_ROTATE_90,
   ID_ROTATE_180,
   ID_ROTATE_270,
   ID_RESUME,
   ID_FLASH_CONTROL
};

class App:public wxApp
{
   public:
      bool OnInit();
};

class Frame:public wxFrame
{
   public:
      Frame(const wxString&);
      ~Frame();

      void CreateUI();
      void Display();

      void OnStartMjpegStream(wxCommandEvent&);
      void OnStopMjpegStream(wxCommandEvent&);

      void OnVerticalFlip(wxCommandEvent&);
      void OnHorizontalFlip(wxCommandEvent&);
      void OnRotate90(wxCommandEvent&);
      void OnRotate180(wxCommandEvent&);
      void OnRotate270(wxCommandEvent&);
      void OnResume(wxCommandEvent&);
      void OnFlashControl(wxCommandEvent&);

      void OnSetResolution(wxCommandEvent&);

      void InitYOLO3();
      void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs);
      void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
      std::vector<cv::String> getOutputsNames(const cv::dnn::Net& yolo_net);

      void InitOpenPose();
      void getKeyPoints(cv::Mat& probMap,double threshold,std::vector<KeyPoint>& keyPoints);
      void splitNetOutputBlobToParts(cv::Mat& netOutputBlob,const cv::Size& targetSize,std::vector<cv::Mat>& netOutputParts);
      void populateColorPalette(std::vector<cv::Scalar>& colors,int nColors);
      void populateInterpPoints(const cv::Point& a,const cv::Point& b,int numPoints,std::vector<cv::Point>& interpCoords);
      void getValidPairs(const std::vector<cv::Mat>& netOutputParts,
	    const std::vector<std::vector<KeyPoint>>& detectedKeypoints,
	    std::vector<std::vector<ValidPair>>& validPairs,
	    std::set<int>& invalidPairs);
      void getPersonwiseKeypoints(const std::vector<std::vector<ValidPair>>& validPairs,
	    const std::set<int>& invalidPairs,
	    std::vector<std::vector<int>>& personwiseKeypoints);


      void OnExit(wxCommandEvent&);
   private:
      wxPanel *screen;
      wxThread *thread;

      ESP32_CAM *esp32_cam;

      wxTextCtrl *ip_ctrl;

      wxButton *start_button;
      wxButton *stop_button;

      wxRadioBox *algorithm_item;

      wxChoice *resolution;

      bool vertical_flag;
      bool horizontal_flag;
      int rotate_select;

      bool flash_flag;

      static const uint32_t IMAGE_WIDTH = 800;
      static const uint32_t IMAGE_HEIGHT = 600;


      // Initialize the parameters
      static constexpr float confThreshold = 0.3f; // Confidence threshold
      static constexpr float nmsThreshold = 0.25f;  // Non-maximum suppression threshold
      static const int inpWidth = 416;//320;  // Width of network's input image
      static const int inpHeight = 416;//320; // Height of network's input image
      std::vector<std::string> classes;
      std::vector<cv::Scalar> classes_color;

      const int nPoints = 18;

      cv::dnn::Net yolo_net;
      cv::dnn::Net openpose_net;

      DECLARE_EVENT_TABLE();
};

class Thread:public wxThread
{
   public:
      Thread(Frame*);

      void* Entry();

   private:
      Frame *frame;
};

   IMPLEMENT_APP(App)
DECLARE_APP(App)

   BEGIN_EVENT_TABLE(Frame,wxFrame)
   EVT_MENU(ID_EXIT,Frame::OnExit)
   EVT_MENU(ID_VERTICAL_FLIP, Frame::OnVerticalFlip)
   EVT_MENU(ID_HORIZONTAL_FLIP, Frame::OnHorizontalFlip)
   EVT_MENU(ID_ROTATE_90, Frame::OnRotate90)
   EVT_MENU(ID_ROTATE_180, Frame::OnRotate180)
   EVT_MENU(ID_ROTATE_270, Frame::OnRotate270)
   EVT_MENU(ID_RESUME, Frame::OnResume)
   EVT_MENU(ID_FLASH_CONTROL, Frame::OnFlashControl)
   EVT_BUTTON(ID_START_MJPEG_STREAM,Frame::OnStartMjpegStream)
   EVT_BUTTON(ID_STOP_MJPEG_STREAM,Frame::OnStopMjpegStream)
   EVT_CHOICE(ID_RESOLUTION, Frame::OnSetResolution)
END_EVENT_TABLE()

bool App::OnInit()
{
   Frame *frame = new Frame(wxT("wxESP32_CAM"));

   frame->Show(true);

   return true;
}

Frame::Frame(const wxString &title):wxFrame(nullptr,wxID_ANY,title,wxDefaultPosition,wxSize(IMAGE_WIDTH+100,IMAGE_HEIGHT+100)/*wxSize(800,600)*/,wxMINIMIZE_BOX | wxCLOSE_BOX | wxCAPTION | wxSYSTEM_MENU)
{
   CreateUI();

   InitYOLO3();

   InitOpenPose();

   esp32_cam = nullptr;
   thread = nullptr;

   vertical_flag = false;
   horizontal_flag = false;
   rotate_select = 0;

   flash_flag = false;
}

Frame::~Frame()
{
   if(thread != nullptr){
      thread->Delete();
      thread = nullptr;
   }

   if(esp32_cam != nullptr){
      esp32_cam->StopVideoStream();
      //delete esp32_cam;
      //esp32_cam = nullptr;
   }   
   Close();
}

void Frame::CreateUI()
{
   wxMenu *file = new wxMenu;

   file->Append(ID_EXIT,wxT("E&xit\tAlt-e"),wxT("Exit"));

   wxMenu *image = new wxMenu;
   image->Append(ID_VERTICAL_FLIP,wxT("Vertical Flip"),wxT("Vertical Flip"));
   image->Append(ID_HORIZONTAL_FLIP,wxT("Horizontal Flip"),wxT("Horizontal Flip"));
   image->AppendSeparator();
   image->Append(ID_ROTATE_90,wxT("Rotate 90"),wxT("Rotate 90"));
   image->Append(ID_ROTATE_180,wxT("Rotate 180"),wxT("Rotate 180"));  
   image->Append(ID_ROTATE_270,wxT("Rotate 270"),wxT("Rotate 270"));
   image->AppendSeparator();
   image->Append(ID_RESUME,wxT("Resume"),wxT("Resume"));

   wxMenu *control = new wxMenu;
   control->Append(ID_FLASH_CONTROL,wxT("Flash On_Off"),wxT("Flash LED On_Off"));

   wxMenuBar *bar = new wxMenuBar;

   bar->Append(file,wxT("File"));
   bar->Append(image,wxT("Image"));
   bar->Append(control,wxT("Control"));
   SetMenuBar(bar);

   wxBoxSizer *top = new wxBoxSizer(wxVERTICAL);
   this->SetSizer(top);

   wxBoxSizer *screen_box = new wxBoxSizer(wxHORIZONTAL);
   top->Add(screen_box,0,wxALIGN_CENTER_HORIZONTAL | wxALL,5);

   screen = new wxPanel(this,ID_SCREEN,wxDefaultPosition,wxSize(IMAGE_WIDTH,IMAGE_HEIGHT));
   screen_box->Add(screen,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,5);

   wxBoxSizer *group1 = new wxBoxSizer(wxHORIZONTAL);

   wxStaticText *ip_lable = new wxStaticText(this,wxID_STATIC,wxT("ESP32-CAM IP: "),wxDefaultPosition,wxDefaultSize,0);
   group1->Add(ip_lable, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 3);
   ip_ctrl = new wxTextCtrl(this,wxID_STATIC,wxT("192.168.110.254"),wxDefaultPosition,wxDefaultSize,0);
   //ip_ctrl = new wxTextCtrl(this,wxID_STATIC,wxT("192.168.1.254"),wxDefaultPosition,wxDefaultSize,0);
   group1->Add(ip_ctrl, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 3);

   start_button = new wxButton(this,ID_START_MJPEG_STREAM,wxT("&Start"),wxDefaultPosition,wxSize(100,35));
   group1->Add(start_button, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 3);
   stop_button = new wxButton(this,ID_STOP_MJPEG_STREAM,wxT("&Stop"),wxDefaultPosition,wxSize(100,35));
   group1->Add(stop_button, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 3);
   start_button->Enable(true);
   stop_button->Enable(false);

   wxArrayString algorithm_item_string;
   algorithm_item_string.Add(wxT("None"));
   algorithm_item_string.Add(wxT("YOLO V3"));
   algorithm_item_string.Add(wxT("OpenPose"));
   algorithm_item = new wxRadioBox(this,wxID_ANY,wxT("Computer Vision Algorithm"),
	 wxDefaultPosition,wxDefaultSize,algorithm_item_string,3,wxRA_SPECIFY_COLS);
   group1->Add(algorithm_item, 0, wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 3);  
   algorithm_item->SetBackgroundColour(wxNullColour);   

   wxArrayString resolution_define;
   resolution_define.Add(wxT("QQVGA 160x120"));
   resolution_define.Add(wxT("QQVGA2 128x160"));
   resolution_define.Add(wxT("QCIF 176x144"));
   resolution_define.Add(wxT("HQVGA 240x176"));
   resolution_define.Add(wxT("QVGA 320x240"));
   resolution_define.Add(wxT("CIF 400x296"));
   resolution_define.Add(wxT("VGA 640x480"));
   resolution_define.Add(wxT("SVGA 800x600"));
   resolution_define.Add(wxT("XGA 1024x768")); 
   resolution_define.Add(wxT("SXGA 1280x1024"));
   resolution_define.Add(wxT("UXGA 1600x1200"));
   //resolution_define.Add(wxT("QXGA 2048*1536"));

   resolution = new wxChoice(this,ID_RESOLUTION,wxDefaultPosition,wxDefaultSize,resolution_define);
   resolution->SetSelection(7);
   group1->Add(resolution,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,5);


   top->Add(group1,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL);

   CreateStatusBar(2);
   SetStatusText(wxT("wxESP32_CAM"));
}

void Frame::OnStartMjpegStream(wxCommandEvent &event)
{
   if(esp32_cam != nullptr){
      delete esp32_cam;
      esp32_cam = nullptr;
   }   

   esp32_cam = new ESP32_CAM(std::string(ip_ctrl->GetValue().mb_str()));

   esp32_cam->StartVideoStream();

   esp32_cam->SetResolution(resolution->GetCurrentSelection());

   if(thread != nullptr){
      thread->Delete();
      thread = nullptr;
   }   
   thread = new Thread(this);
   thread->Create();
   thread->Run(); 

   start_button->Enable(false);
   stop_button->Enable(true);
   ip_ctrl->Enable(false);
}

void Frame::OnStopMjpegStream(wxCommandEvent &event)
{
   if(esp32_cam != nullptr){
      esp32_cam->StopVideoStream();
      //delete esp32_cam;
      //esp32_cam = nullptr;
   }

   if(thread != nullptr){
      thread->Delete();
      thread = nullptr;
   }
   start_button->Enable(true);
   stop_button->Enable(false);  
   ip_ctrl->Enable(true);
}

void Frame::OnSetResolution(wxCommandEvent &event)
{
   /*
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
      */

   if(esp32_cam != nullptr){
      esp32_cam->SetResolution(resolution->GetCurrentSelection());
   }
}

void Frame::OnVerticalFlip(wxCommandEvent &event)
{
   vertical_flag ^= true;
}

void Frame::OnHorizontalFlip(wxCommandEvent &event)
{
   horizontal_flag ^= true;
}

void Frame::OnRotate90(wxCommandEvent &event)
{
   rotate_select = 1;
}

void Frame::OnRotate180(wxCommandEvent &event)
{
   rotate_select = 2;
}

void Frame::OnRotate270(wxCommandEvent &event)
{
   rotate_select = 3;
}

void Frame::OnResume(wxCommandEvent &event)
{
   vertical_flag = false;
   horizontal_flag = false;
   rotate_select = 0;
}

void Frame::OnFlashControl(wxCommandEvent &event)
{
   flash_flag ^= true;
   esp32_cam->FlashControl(flash_flag);
}


void Frame::Display()
{
   wxString str;

   std::string rssi = esp32_cam->GetRSSI();

   str.Printf(wxT(" ,JPEG Size: %.2fKb ,RSSI:%s ,Opencv%d"),(esp32_cam->GetFrameSize() / 1024.0f),rssi,CV_MAJOR_VERSION);
   SetStatusText(wxDateTime::Now().Format() + str);


   cv::Mat src,dst,blob;
   //src = esp32_cam->GetFrame();
   //*esp32_cam >> src;
   esp32_cam >> src;
   if(src.empty()){
      return;
   }

   if(vertical_flag){
      cv::flip(src,src,0);
   }

   if(horizontal_flag){
      cv::flip(src,src,1);
   }

   if(rotate_select == 1){
      cv::rotate(src,src,cv::ROTATE_90_CLOCKWISE);
   }
   else if(rotate_select == 2){
      cv::rotate(src,src,cv::ROTATE_180);
   }
   else if(rotate_select == 3){
      cv::rotate(src,src,cv::ROTATE_90_COUNTERCLOCKWISE);
   }

   if(algorithm_item->GetSelection() == 0){
      cv::cvtColor(src,dst,cv::COLOR_RGB2BGR/*CV_RGB2BGR*/);
   }
   else if(algorithm_item->GetSelection() == 1){
      // Create a 4D blob from a frame.
      cv::dnn::blobFromImage(src, blob, 1/255.0, cvSize(inpWidth, inpHeight), cv::Scalar(0,0,0), true, false);

      //Sets the input to the network
      yolo_net.setInput(blob);

      // Runs the forward pass to get output of the output layers
      std::vector<cv::Mat> outs;
      yolo_net.forward(outs, getOutputsNames(yolo_net));

      // Remove the bounding boxes with low confidence
      postprocess(src, outs);   

      // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
      std::vector<double> layersTimes;
      double freq = cv::getTickFrequency() / 1000;
      double t = yolo_net.getPerfProfile(layersTimes) / freq;
      std::string label = cv::format("Inference time for a frame : %.2f ms", t);
      cv::putText(src, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));	

      cv::cvtColor(src,dst,cv::COLOR_RGB2BGR/*CV_RGB2BGR*/);
   }
   else if(algorithm_item->GetSelection() == 2){
      cv::Mat inputBlob = cv::dnn::blobFromImage(src,1.0/255.0,cv::Size((int)((368*src.cols)/src.rows),368),cv::Scalar(0,0,0),false,false);

      std::chrono::time_point<std::chrono::system_clock> startTP = std::chrono::system_clock::now();

      openpose_net.setInput(inputBlob);

      cv::Mat netOutputBlob = openpose_net.forward();

      std::vector<cv::Mat> netOutputParts;
      splitNetOutputBlobToParts(netOutputBlob,cv::Size(src.cols,src.rows),netOutputParts);

      std::chrono::time_point<std::chrono::system_clock> finishTP = std::chrono::system_clock::now();

      std::string label = cv::format("Inference time for a frame : %lld ms", std::chrono::duration_cast<std::chrono::milliseconds>(finishTP - startTP).count());
      cv::putText(src, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));		 

      int keyPointId = 0;
      std::vector<std::vector<KeyPoint>> detectedKeypoints;
      std::vector<KeyPoint> keyPointsList;     

      for(int i = 0; i < nPoints;++i){
	 std::vector<KeyPoint> keyPoints;

	 getKeyPoints(netOutputParts[i],0.1,keyPoints);

	 for(int i = 0; i< keyPoints.size();++i,++keyPointId){
	    keyPoints[i].id = keyPointId;
	 }

	 detectedKeypoints.push_back(keyPoints);
	 keyPointsList.insert(keyPointsList.end(),keyPoints.begin(),keyPoints.end());
      }     

      std::vector<cv::Scalar> colors;
      populateColorPalette(colors,nPoints);

      cv::Mat outputFrame = src.clone();

      for(int i = 0; i < nPoints;++i){
	 for(int j = 0; j < detectedKeypoints[i].size();++j){
	    cv::circle(outputFrame,detectedKeypoints[i][j].point,5,colors[i],-1,cv::LINE_AA);
	 }
      }     

      std::vector<std::vector<ValidPair>> validPairs;
      std::set<int> invalidPairs;
      getValidPairs(netOutputParts,detectedKeypoints,validPairs,invalidPairs);

      std::vector<std::vector<int>> personwiseKeypoints;
      getPersonwiseKeypoints(validPairs,invalidPairs,personwiseKeypoints);

      for(int i = 0; i< nPoints-1;++i){
	 for(int n  = 0; n < personwiseKeypoints.size();++n){
	    const std::pair<int,int>& posePair = posePairs[i];
	    int indexA = personwiseKeypoints[n][posePair.first];
	    int indexB = personwiseKeypoints[n][posePair.second];

	    if(indexA == -1 || indexB == -1){
	       continue;
	    }

	    const KeyPoint& kpA = keyPointsList[indexA];
	    const KeyPoint& kpB = keyPointsList[indexB];

	    cv::line(outputFrame,kpA.point,kpB.point,colors[i],3,cv::LINE_AA);

	 }
      }    

      cv::cvtColor(outputFrame,dst,cv::COLOR_RGB2BGR/*CV_RGB2BGR*/);

   }

   cv::Size size = dst.size();
   wxImage *image = new wxImage(size.width,size.height,dst.data,true);
   image->Rescale(IMAGE_WIDTH,IMAGE_HEIGHT);

   wxBitmap *bitmap = new wxBitmap(*image);

   int x,y,width,height;

   wxClientDC dc(screen);
   dc.GetClippingBox(&x,&y,&width,&height);
   dc.DrawBitmap(*bitmap,x,y);

   delete bitmap;
   delete image;
}

void Frame::InitYOLO3()
{
   wxString str;
   wxStandardPaths& sp = wxStandardPaths::Get();
   str.Printf("%s/coco.names",sp.GetResourcesDir());
   // Load names of classes
   std::string classesFile(str.mb_str());
   std::ifstream ifs(classesFile.c_str());
   std::string line;

   std::random_device rd;
   std::default_random_engine gen = std::default_random_engine(rd());
   std::uniform_int_distribution<int> dis(0,255);

   while(getline(ifs, line)){
      classes.push_back(line);

      classes_color.push_back(cv::Scalar(dis(gen),dis(gen),dis(gen)));
   }

   // Give the configuration and weight files for the model
   str.Printf("%s/yolov3.cfg",sp.GetResourcesDir());
   //str.Printf("%s/yolov3-tiny.cfg",sp.GetResourcesDir());
   cv::String modelConfiguration(str.mb_str());
   str.Printf("%s/yolov3.weights",sp.GetResourcesDir());
   //str.Printf("%s/yolov3-tiny.weights",sp.GetResourcesDir());
   cv::String modelWeights(str.mb_str());

   // Load the network
   yolo_net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
   yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
   yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);   
}

// Remove the bounding boxes with low confidence using non-maxima suppression
void Frame::postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs)
{
   std::vector<int> classIds;
   std::vector<float> confidences;
   std::vector<cv::Rect> boxes;

   for (size_t i = 0; i < outs.size(); ++i)
   {
      // Scan through all the bounding boxes output from the network and keep only the
      // ones with high confidence scores. Assign the box's class label as the class
      // with the highest score for the box.
      float* data = (float*)outs[i].data;
      for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
      {
	 cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
	 cv::Point classIdPoint;
	 double confidence;
	 // Get the value and location of the maximum score
	 minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
	 if (confidence > confThreshold)
	 {
	    int centerX = (int)(data[0] * frame.cols);
	    int centerY = (int)(data[1] * frame.rows);
	    int width = (int)(data[2] * frame.cols);
	    int height = (int)(data[3] * frame.rows);
	    int left = centerX - width / 2;
	    int top = centerY - height / 2;

	    classIds.push_back(classIdPoint.x);
	    confidences.push_back((float)confidence);
	    boxes.push_back(cv::Rect(left, top, width, height));
	 }
      }
   }

   // Perform non maximum suppression to eliminate redundant overlapping boxes with
   // lower confidences
   std::vector<int> indices;
   cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
   for (size_t i = 0; i < indices.size(); ++i)
   {
      int idx = indices[i];
      cv::Rect box = boxes[idx];
      drawPred(classIds[idx], confidences[idx], box.x, box.y,
	    box.x + box.width, box.y + box.height, frame);
   }
}

// Draw the predicted bounding box
void Frame::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
{
   //Draw a rectangle displaying the bounding box
   //cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);

   //Get the label for the class name and its confidence
   std::string label = cv::format("%.2f", conf);
   if (!classes.empty())
   {
      CV_Assert(classId < (int)classes.size());
      label = classes[classId] + ":" + label;
      //Draw a rectangle displaying the bounding box
      cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), classes_color[classId], 3);
   }

   //Display the label at the top of the bounding box
   int baseLine;
   cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
   top = cv::max(top, labelSize.height);
   cv::rectangle(frame, cv::Point(left, top - round(1.5*labelSize.height)), cv::Point(left + round(1.5*labelSize.width), top + baseLine), classes_color[classId]/*cv::Scalar(255, 255, 255)*/, cv::FILLED);
   cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,0),1);
}

// Get the names of the output layers
std::vector<cv::String> Frame::getOutputsNames(const cv::dnn::Net& yolo_net)
{
   static std::vector<cv::String> names;
   if (names.empty())
   {
      //Get the indices of the output layers, i.e. the layers with unconnected outputs
      std::vector<int> outLayers = yolo_net.getUnconnectedOutLayers();

      //get the names of all the layers in the network
      std::vector<cv::String> layersNames = yolo_net.getLayerNames();

      // Get the names of the output layers in names
      names.resize(outLayers.size());
      for (size_t i = 0; i < outLayers.size(); ++i)
	 names[i] = layersNames[outLayers[i] - 1];
   }
   return names;
}

void Frame::InitOpenPose()
{
   wxString str1,str2;
   wxStandardPaths& sp = wxStandardPaths::Get();
   str1.Printf("%s/pose_deploy_linevec.prototxt",sp.GetResourcesDir());
   str2.Printf("%s/pose_iter_440000.caffemodel",sp.GetResourcesDir());

   openpose_net = cv::dnn::readNetFromCaffe((const char*)str1.c_str(),(const char*)str2.c_str());   
}

void Frame::getKeyPoints(cv::Mat& probMap,double threshold,std::vector<KeyPoint>& keyPoints){
   cv::Mat smoothProbMap;
   cv::GaussianBlur( probMap, smoothProbMap, cv::Size( 3, 3 ), 0, 0 );

   cv::Mat maskedProbMap;
   cv::threshold(smoothProbMap,maskedProbMap,threshold,255,cv::THRESH_BINARY);

   maskedProbMap.convertTo(maskedProbMap,CV_8U,1);

   std::vector<std::vector<cv::Point> > contours;
   cv::findContours(maskedProbMap,contours,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);

   for(int i = 0; i < contours.size();++i){
      cv::Mat blobMask = cv::Mat::zeros(smoothProbMap.rows,smoothProbMap.cols,smoothProbMap.type());

      cv::fillConvexPoly(blobMask,contours[i],cv::Scalar(1));

      double maxVal;
      cv::Point maxLoc;

      cv::minMaxLoc(smoothProbMap.mul(blobMask),0,&maxVal,0,&maxLoc);

      keyPoints.push_back(KeyPoint(maxLoc, probMap.at<float>(maxLoc.y,maxLoc.x)));
   }
}

void Frame::splitNetOutputBlobToParts(cv::Mat& netOutputBlob,const cv::Size& targetSize,std::vector<cv::Mat>& netOutputParts){
   int nParts = netOutputBlob.size[1];
   int h = netOutputBlob.size[2];
   int w = netOutputBlob.size[3];

   for(int i = 0; i< nParts;++i){
      cv::Mat part(h, w, CV_32F, netOutputBlob.ptr(0,i));

      cv::Mat resizedPart;

      cv::resize(part,resizedPart,targetSize);

      netOutputParts.push_back(resizedPart);
   }
}

void Frame::populateColorPalette(std::vector<cv::Scalar>& colors,int nColors){
   std::random_device rd;
   std::mt19937 gen(rd());
   std::uniform_int_distribution<> dis1(64, 200);
   std::uniform_int_distribution<> dis2(100, 255);
   std::uniform_int_distribution<> dis3(100, 255);

   for(int i = 0; i < nColors;++i){
      colors.push_back(cv::Scalar(dis1(gen),dis2(gen),dis3(gen)));
   }
}

void Frame::populateInterpPoints(const cv::Point& a,const cv::Point& b,int numPoints,std::vector<cv::Point>& interpCoords){
   float xStep = ((float)(b.x - a.x))/(float)(numPoints-1);
   float yStep = ((float)(b.y - a.y))/(float)(numPoints-1);

   interpCoords.push_back(a);

   for(int i = 1; i< numPoints-1;++i){
      interpCoords.push_back(cv::Point(a.x + xStep*i,a.y + yStep*i));
   }

   interpCoords.push_back(b);
}

void Frame::getValidPairs(const std::vector<cv::Mat>& netOutputParts,
      const std::vector<std::vector<KeyPoint>>& detectedKeypoints,
      std::vector<std::vector<ValidPair>>& validPairs,
      std::set<int>& invalidPairs) {

   int nInterpSamples = 10;
   float pafScoreTh = 0.1;
   float confTh = 0.7;

   for(int k = 0; k < mapIdx.size();++k ){

      //A->B constitute a limb
      cv::Mat pafA = netOutputParts[mapIdx[k].first];
      cv::Mat pafB = netOutputParts[mapIdx[k].second];

      //Find the keypoints for the first and second limb
      const std::vector<KeyPoint>& candA = detectedKeypoints[posePairs[k].first];
      const std::vector<KeyPoint>& candB = detectedKeypoints[posePairs[k].second];

      int nA = candA.size();
      int nB = candB.size();

      /*
# If keypoints for the joint-pair is detected
# check every joint in candA with every joint in candB
# Calculate the distance vector between the two joints
# Find the PAF values at a set of interpolated points between the joints
# Use the above formula to compute a score to mark the connection valid
*/

      if(nA != 0 && nB != 0){
	 std::vector<ValidPair> localValidPairs;

	 for(int i = 0; i< nA;++i){
	    int maxJ = -1;
	    float maxScore = -1;
	    bool found = false;

	    for(int j = 0; j < nB;++j){
	       std::pair<float,float> distance(candB[j].point.x - candA[i].point.x,candB[j].point.y - candA[i].point.y);

	       float norm = std::sqrt(distance.first*distance.first + distance.second*distance.second);

	       if(!norm){
		  continue;
	       }

	       distance.first /= norm;
	       distance.second /= norm;

	       //Find p(u)
	       std::vector<cv::Point> interpCoords;
	       populateInterpPoints(candA[i].point,candB[j].point,nInterpSamples,interpCoords);
	       //Find L(p(u))
	       std::vector<std::pair<float,float>> pafInterp;
	       for(int l = 0; l < interpCoords.size();++l){
		  pafInterp.push_back(
			std::pair<float,float>(
			   pafA.at<float>(interpCoords[l].y,interpCoords[l].x),
			   pafB.at<float>(interpCoords[l].y,interpCoords[l].x)
			   ));
	       }

	       std::vector<float> pafScores;
	       float sumOfPafScores = 0;
	       int numOverTh = 0;
	       for(int l = 0; l< pafInterp.size();++l){
		  float score = pafInterp[l].first*distance.first + pafInterp[l].second*distance.second;
		  sumOfPafScores += score;
		  if(score > pafScoreTh){
		     ++numOverTh;
		  }

		  pafScores.push_back(score);
	       }

	       float avgPafScore = sumOfPafScores/((float)pafInterp.size());

	       if(((float)numOverTh)/((float)nInterpSamples) > confTh){
		  if(avgPafScore > maxScore){
		     maxJ = j;
		     maxScore = avgPafScore;
		     found = true;
		  }
	       }

	    }/* j */

	    if(found){
	       localValidPairs.push_back(ValidPair(candA[i].id,candB[maxJ].id,maxScore));
	    }

	 }/* i */

	 validPairs.push_back(localValidPairs);

      } else {
	 invalidPairs.insert(k);
	 validPairs.push_back(std::vector<ValidPair>());
      }
   }/* k */
}

void Frame::getPersonwiseKeypoints(const std::vector<std::vector<ValidPair>>& validPairs,
      const std::set<int>& invalidPairs,
      std::vector<std::vector<int>>& personwiseKeypoints) {
   for(int k = 0; k < mapIdx.size();++k){
      if(invalidPairs.find(k) != invalidPairs.end()){
	 continue;
      }

      const std::vector<ValidPair>& localValidPairs(validPairs[k]);

      int indexA(posePairs[k].first);
      int indexB(posePairs[k].second);

      for(int i = 0; i< localValidPairs.size();++i){
	 bool found = false;
	 int personIdx = -1;

	 for(int j = 0; !found && j < personwiseKeypoints.size();++j){
	    if(indexA < personwiseKeypoints[j].size() &&
		  personwiseKeypoints[j][indexA] == localValidPairs[i].aId){
	       personIdx = j;
	       found = true;
	    }
	 }/* j */

	 if(found){
	    personwiseKeypoints[personIdx].at(indexB) = localValidPairs[i].bId;
	 } else if(k < 17){
	    std::vector<int> lpkp(std::vector<int>(18,-1));

	    lpkp.at(indexA) = localValidPairs[i].aId;
	    lpkp.at(indexB) = localValidPairs[i].bId;

	    personwiseKeypoints.push_back(lpkp);
	 }

      }/* i */
   }/* k */
}

void Frame::OnExit(wxCommandEvent &event)
{
   Close();
}

Thread::Thread(Frame *parent):wxThread(wxTHREAD_DETACHED)
{
   frame = parent;
}

void* Thread::Entry()
{
   while(!TestDestroy()){
      //wxMutexGuiEnter();
      frame->Display();
      //wxMutexGuiLeave();
      //Sleep(25);
   }

   return nullptr;
}
