CXX = g++
CXXFLAGS = -Iimgui -Iimplot -Ieigen -IAdafruit_AHRS/src -O2 -Wall
LDFLAGS = -lglfw -lGL

# Source files
IMGUI_SRCS = imgui/imgui.cpp imgui/imgui_draw.cpp imgui/imgui_tables.cpp imgui/imgui_widgets.cpp imgui/imgui_demo.cpp \
             imgui/backends/imgui_impl_glfw.cpp imgui/backends/imgui_impl_opengl3.cpp
IMPLOT_SRCS = implot/implot.cpp implot/implot_items.cpp implot/implot_demo.cpp
MAIN_SRCS = main.cpp
MAIN_HEADERS = fsm.hpp KalmanFilter.hpp
AHRS_SRCS = Adafruit_AHRS/src/Adafruit_AHRS_Madgwick.cpp Adafruit_AHRS/src/Adafruit_AHRS_Mahony.cpp

# Object files
IMGUI_OBJS = $(IMGUI_SRCS:.cpp=.o)
IMPLOT_OBJS = $(IMPLOT_SRCS:.cpp=.o)
MAIN_OBJS = $(MAIN_SRCS:.cpp=.o)
AHRS_OBJS = $(AHRS_SRCS:.cpp=.o)

ALL_OBJS = $(IMGUI_OBJS) $(IMPLOT_OBJS) $(MAIN_OBJS) $(AHRS_OBJS)

# Target
TARGET = plotter

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(ALL_OBJS)
	$(CXX) $^ $(LDFLAGS) -o $@

# ImGui objects
imgui/%.o: imgui/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

imgui/backends/%.o: imgui/backends/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# ImPlot objects
implot/%.o: implot/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Main object
%.o: %.cpp $(MAIN_HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Adafruit AHRS objects
Adafruit_AHRS/%.o: Adafruit_AHRS/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(ALL_OBJS) $(TARGET)
