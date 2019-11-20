all:
	g++ -o3 -o wxESP32-CAM.app wxESP32-CAM.cpp ESP32-CAM\ Library.cpp `wx-config --cxxflags --libs` `pkg-config opencv4 --cflags --libs` -lpthread -lcurl -std=c++11 -m64 -D _MAC_
	./wxESP32-CAM.app
