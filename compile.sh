#!/bin/bash
cd "$(dirname "$0")"
g++ main.cpp imgui/imgui*.cpp imgui/backends/imgui_impl_glfw.cpp imgui/backends/imgui_impl_opengl3.cpp implot/implot*.cpp Adafruit_AHRS/src/Adafruit_AHRS_Madgwick.cpp Adafruit_AHRS/src/Adafruit_AHRS_Mahony.cpp -Iimgui -Iimplot -Ieigen -IAdafruit_AHRS/src -lglfw -lGL -o main_export_09182026_022604/a.out
