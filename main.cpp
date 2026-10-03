#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>
#include <filesystem>
#include <cmath>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "implot.h"
#include <GLFW/glfw3.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

#include "KalmanFilter.hpp"
#include "Adafruit_AHRS_Mahony.h"
#include "barometer.h"


// State filter for orientation estimation, used in the IMU task
Adafruit_Mahony orientation;

// Altitude and veritcal velocity state filter, used to sense when to deploy
// the parachute
KalmanFilter altitude;


std::filesystem::path get_current_dir() {
    return std::filesystem::canonical("/proc/self/exe").parent_path();
}

std::vector<std::string> get_available_ports() {
    std::vector<std::string> ports;
    std::string dev_dir = "/dev/";
    try {
        for (const auto & entry : std::filesystem::directory_iterator(dev_dir)) {
            std::string filename = entry.path().filename().string();
            if (filename.find("ttyACM") == 0 || filename.find("ttyUSB") == 0) {
                ports.push_back(entry.path().string());
            }
        }
    } catch (...) {}
    return ports;
}

int init_serial(const char* portname) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        return -1;
    }
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return -1;
    }
    return fd;
}

int main(int argc, char** argv){
    std::filesystem::path data_dir = std::filesystem::current_path();

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-d") {
            if (i + 1 < argc) {
                data_dir = argv[++i];
            } else {
                std::cerr << "Errore: l'opzione -d richiede un percorso di cartella\n";
                return 1;
            }
        }
    }

    std::fstream file_imu(data_dir / "imu.csv", std::ios::in);
    std::fstream file_baro(data_dir / "baro.csv", std::ios::in);
    std::fstream file_filtered(data_dir / "filteredDataInfo.csv", std::ios::in);

    if (!file_imu.is_open() || !file_baro.is_open() || !file_filtered.is_open()) {
        std::cerr << "File imu.csv, baro.csv o filteredDataInfo.csv non trovati in " << data_dir << "\n";
        return 1;
    }

    std::string header = "ts,Ax,Ay,Az,Gx,Gy,Gz,P,filteredAltitudeAGL,filteredAcceleration\n";
    std::string line_imu, line_baro, line_filtered;

    // Read the headers first
    if (!std::getline(file_imu, line_imu) || !std::getline(file_baro, line_baro) || !std::getline(file_filtered, line_filtered)) {
        std::cerr << "Almeno un file è vuoto\n";
        return 1;
    }

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return 1;
    }

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(1280, 720, "Live Plots", NULL, NULL);
    if (window == NULL) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    // Setup ImGui Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    std::filesystem::path current_dir = get_current_dir();

    std::vector<float> ts_data;
    std::vector<float> alt_data;
    std::vector<float> acc_data;
    float current_ts = 0.0f;

    auto last_time = std::chrono::steady_clock::now();
    bool is_started = false;
    bool is_paused = false;
    int serial_fd = -1;
    std::vector<std::string> available_ports = get_available_ports();
    int selected_port_idx = 0;
    auto last_port_refresh_time = std::chrono::steady_clock::now();


    orientation.begin(100);
    altitude.setG(9.80665);

    std::vector<float> altitude_filtered_data;
    std::vector<float> delta_data;
    float P0 = 0.0f;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        auto current_time = std::chrono::steady_clock::now();

        if (is_started) {
            if (is_paused) {
                last_time = current_time;
            } else {
                // Read lines to catch up with 10ms per line
                while (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count() >= 10) {
                    last_time += std::chrono::milliseconds(10);

                if (std::getline(file_imu, line_imu) && std::getline(file_baro, line_baro) && std::getline(file_filtered, line_filtered)) {

                    std::stringstream ss_imu(line_imu);
                    std::stringstream ss_baro(line_baro);
                    std::stringstream ss_filtered(line_filtered);
                    std::stringstream ss_out;

                    std::string value_imu, value_baro, value_filtered;

                    std::vector<float> imu_values(6, 0.0f); // Ax, Ay, Az, Gx, Gy, Gz
                    for (int i = 0; std::getline(ss_imu, value_imu, ',' ); i++) {
                        if (i == 0) {
                            current_ts = std::stof(value_imu); // Extract ts
                            ss_out << value_imu << ",";
                        } else if (i == 1) {
                            continue; // skip ID
                        } else {
                            if (i - 2 < imu_values.size()) {
                                imu_values[i-2] = std::stof(value_imu);
                            }
                            ss_out << value_imu << ",";
                        }
                    }

                    float baro_pressure = 0.0f;
                    float baro_temp = 0.0f;
                    for (int i = 0; std::getline(ss_baro, value_baro, ',' ); i++) {
                        if (i == 2) {
                            baro_temp = std::stof(value_baro);
                        }
                        if (i == 3){
                            baro_pressure = std::stof(value_baro);
                            ss_out << value_baro << ",";
                        }
                    }

                    float alt_filtered = 0.0f, acc_filtered = 0.0f;
                    for (int i = 0; std::getline(ss_filtered, value_filtered, ',' ); i++) {
                        if (i == 1) alt_filtered = std::stof(value_filtered); // Extract filteredAltitudeAGL
                        if (i == 2) acc_filtered = std::stof(value_filtered); // Extract filteredAcceleration

                        if (i != 0)
                            ss_out << "," << value_filtered << (i == 1 ? "," : "");
                    }

                    ss_out << '\n';
                    std::string out_str = ss_out.str();

                    std::cout << out_str << std::flush;
                    if (serial_fd != -1) {
                        write(serial_fd, out_str.c_str(), out_str.length());
                    }

                    ts_data.push_back(current_ts);
                    alt_data.push_back(alt_filtered);
                    acc_data.push_back(acc_filtered);

                    // MAPPING THE AXES:
                    // The rocket's UP axis is -Y (Ay is -9.90 on the pad, -86 during launch)
                    // Adafruit_AHRS expects gravity on +Z (Z is DOWN).
                    // So we map the rocket's +Y (DOWN) to the filter's +Z (DOWN).
                    // To keep it right-handed: X' = X, Y' = -Z, Z' = Y
                    float ax = imu_values[0];
                    float ay = -imu_values[2];
                    float az = imu_values[1];

                    float gx = imu_values[3];
                    float gy = -imu_values[5];
                    float gz = imu_values[4];

                    orientation.updateIMU(gx, gy, gz, ax, ay, az);

                    // The tilt from vertical is the tilt from the filter's Z axis
                    float attitude_rad = acos(cos(orientation.getPitchRadians())*cos(orientation.getRollRadians()));

                    // Longitudinal acceleration is -Ay (since rocket accelerates in -Y direction)
                    float longitudinal_accel = -imu_values[1];

                    altitude.predict(
                        longitudinal_accel,
                        attitude_rad,
                        false
                    );

                    // Initialize P0 with the first pressure reading
                    if (P0 == 0.0f && baro_pressure > 0.0f) {
                        P0 = baro_pressure;

                        //IMPORTANTE: il Vega sembra assumere baro_temp = 15 e costante
                        ground_temperature_k = baro_temp + 273.15f; // assuming celsius
                    }

                    // Convert pressure to altitude using compute_altitude from barometer.h
                    float alt_baro = 0.0f;
                    if (P0 > 0.0f) {
                        alt_baro = compute_altitude(baro_pressure, P0);
                    }

                    altitude.update(alt_baro);

                    altitude_filtered_data.push_back(altitude.getState()[0]);
                    delta_data.push_back(altitude.getState()[0] - alt_filtered);

                }
            }
        }
    }

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::Begin("Live Plots", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings);

        if (!is_started) {
            auto current_refresh_time = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(current_refresh_time - last_port_refresh_time).count() >= 1000) {
                last_port_refresh_time = current_refresh_time;
                std::string current_selection = "";
                if (!available_ports.empty() && selected_port_idx < available_ports.size()) {
                    current_selection = available_ports[selected_port_idx];
                }
                available_ports = get_available_ports();
                selected_port_idx = 0;
                for (int i = 0; i < available_ports.size(); i++) {
                    if (available_ports[i] == current_selection) {
                        selected_port_idx = i;
                        break;
                    }
                }
            }

            ImGui::Text("Select Serial Port (Auto-updating):");
            std::string preview_value = available_ports.empty() ? "No ports found" : available_ports[selected_port_idx];

            ImGui::PushItemWidth(250);
            if (ImGui::BeginCombo("##Serial Port", preview_value.c_str())) {
                for (int i = 0; i < available_ports.size(); i++) {
                    const bool is_selected = (selected_port_idx == i);
                    if (ImGui::Selectable(available_ports[i].c_str(), is_selected)) {
                        selected_port_idx = i;
                    }
                    if (is_selected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::PopItemWidth();

            ImGui::Separator();

            if (ImGui::Button("START STREAMING AND PLOTTING", ImVec2(300, 50))) {
                if (!available_ports.empty()) {
                    serial_fd = init_serial(available_ports[selected_port_idx].c_str());
                    if (serial_fd == -1) {
                        std::cerr << "Failed to open serial port " << available_ports[selected_port_idx] << "\n";
                    }
                }

                std::cout << header << std::flush;
                if (serial_fd != -1) {
                    write(serial_fd, header.c_str(), header.length());
                }

                is_started = true;
                last_time = std::chrono::steady_clock::now(); // Reset timing
            }
        } else {
            ImGui::Text("Time (ts): %.3f s", current_ts);
            ImGui::SameLine();
            if (is_paused) {
                if (ImGui::Button("RESUME")) {
                    is_paused = false;
                    last_time = std::chrono::steady_clock::now();
                }
            } else {
                if (ImGui::Button("PAUSE")) {
                    is_paused = true;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("RESTART")) {
                ts_data.clear();
                alt_data.clear();
                acc_data.clear();
                altitude_filtered_data.clear(); // FIX: Clear this array so it doesn't cause out-of-bounds segfaults
                delta_data.clear();
                current_ts = 0.0f;
                P0 = 0.0f; // Reset pressure reference

                file_imu.clear(); file_imu.seekg(0);
                file_baro.clear(); file_baro.seekg(0);
                file_filtered.clear(); file_filtered.seekg(0);

                std::getline(file_imu, line_imu);
                std::getline(file_baro, line_baro);
                std::getline(file_filtered, line_filtered);

                if (serial_fd != -1) {
                    write(serial_fd, header.c_str(), header.length());
                }

                last_time = std::chrono::steady_clock::now();
                is_paused = false;
            }
            ImGui::SameLine(ImGui::GetWindowWidth() - 150);
            if (ImGui::Button("STOP", ImVec2(100, 0))) {
                is_started = false;
                is_paused = false;
                if (serial_fd != -1) {
                    close(serial_fd);
                    serial_fd = -1;
                }
            }

            ImGui::Separator();

            if (!ts_data.empty()) {
                ImVec2 plot_size = ImVec2(-1, 300);
                if (ImPlot::BeginPlot("Altitude", plot_size)) {
                    ImPlot::SetupAxes("Time (s)", "Altitude AGL (m)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotLine("filteredAltitudeAGL", ts_data.data(), alt_data.data(), ts_data.size());
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Altitude StarFly", plot_size)) {
                    ImPlot::SetupAxes("Time (s)", "Altitude AGL (m)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotLine("filteredAltitudeAGL", ts_data.data(), altitude_filtered_data.data(), ts_data.size());
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Delta (Starfly - CSV)", plot_size)) {
                    ImPlot::SetupAxes("Time (s)", "Delta (m)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotLine("Delta", ts_data.data(), delta_data.data(), ts_data.size());
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Acceleration", plot_size)) {
                    ImPlot::SetupAxes("Time (s)", "Acceleration (m/s^2)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotLine("filteredAcceleration", ts_data.data(), acc_data.data(), ts_data.size());
                    ImPlot::EndPlot();
                }
            }
        }

        ImGui::End();

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    if (serial_fd != -1) {
        close(serial_fd);
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
