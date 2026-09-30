#!/bin/bash
cd "$(dirname "$0")"
g++ main.cpp imgui/imgui*.cpp imgui/backends/imgui_impl_glfw.cpp imgui/backends/imgui_impl_opengl3.cpp implot/implot*.cpp -Iimgui -Iimplot -lglfw -lGL -o main_export_09182026_022604/a.out
