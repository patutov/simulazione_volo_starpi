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
#include "fsm.hpp"
#include "sensor_health.hpp"

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

void print_help(const char* prog_name, std::ostream& os = std::cout) {
    os << "Uso: " << prog_name << " [-d <cartella_dati>] [-s | --serial]\n\n"
       << "Opzioni:\n"
       << "  -d <cartella_dati>   Specifica la cartella contenente i file CSV\n"
       << "                       (imu.csv, baro.csv, filteredDataInfo.csv).\n"
       << "                       Default: cartella corrente\n"
       << "  -s, --serial         Abilita l'output seriale (disabilitato di default)\n"
       << "  -h, --help           Mostra questo messaggio di aiuto ed esce\n";
}

int main(int argc, char** argv){
    std::filesystem::path data_dir = std::filesystem::current_path();
    bool enable_serial = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-d") {
            if (i + 1 < argc) {
                data_dir = argv[++i];
            } else {
                std::cerr << "Errore: l'opzione -d richiede un percorso di cartella\n\n";
                print_help(argv[0], std::cerr);
                return 1;
            }
        } else if (arg == "-s" || arg == "--serial" || arg == "--enable-serial") {
            enable_serial = true;
        } else if (arg == "-h" || arg == "--help") {
            print_help(argv[0], std::cout);
            return 0;
        } else {
            std::cerr << "Opzione non riconosciuta: " << arg << "\n\n";
            print_help(argv[0], std::cerr);
            return 1;
        }
    }

    std::fstream file_imu(data_dir / "imu.csv", std::ios::in);
    std::fstream file_baro(data_dir / "baro.csv", std::ios::in);
    std::fstream file_filtered(data_dir / "filteredDataInfo.csv", std::ios::in);

    if (!file_imu.is_open() || !file_baro.is_open() || !file_filtered.is_open()) {
        std::cerr << "File imu.csv, baro.csv o filteredDataInfo.csv non trovati in " << data_dir << "\n\n";
        print_help(argv[0], std::cerr);
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
    altitude.setG(1.0);

    std::vector<float> altitude_filtered_data;
    std::vector<float> delta_data;
    std::vector<float> vspeed_data;
    std::vector<std::pair<float, std::string>> state_changes;
    std::vector<std::pair<float, std::string>> fault_changes;
    RocketState last_rocket_state = RS_IDLE;
    float P0 = 0.0f;
    bool imu_healthy = true;
    bool baro_healthy = true;

    // TODO: init orientation filter with first value of quaternions from orientationInfo.csv

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
                            if ((size_t)i - 2 < imu_values.size()) {
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

                    float alt_vega = 0.0f, acc_filtered = 0.0f;
                    for (int i = 0; std::getline(ss_filtered, value_filtered, ',' ); i++) {
                        if (i == 1) alt_vega = std::stof(value_filtered); // Extract filteredAltitudeAGL
                        if (i == 2) acc_filtered = std::stof(value_filtered); // Extract filteredAcceleration

                        if (i != 0)
                            ss_out << "," << value_filtered << (i == 1 ? "," : "");
                    }

                    ss_out << '\n';
                    std::string out_str = ss_out.str();

                    if (enable_serial) {
                        std::cout << out_str << std::flush;
                        if (serial_fd != -1) {
                            write(serial_fd, out_str.c_str(), out_str.length());
                        }
                    }

                    ts_data.push_back(current_ts);
                    alt_data.push_back(alt_vega);
                    acc_data.push_back(acc_filtered/9.81f);


                    // MAPPING THE AXES:
                    // Acceleration in G
                    float ax =  imu_values[0] / 9.81f;
                    float ay =  imu_values[2] / 9.81f;
                    float az =  imu_values[1] / 9.81f;
                    // rotation in DPS
                    float gx =  imu_values[3];
                    float gy =  imu_values[5];
                    float gz =  imu_values[4];

                    // HEALTH CHECK
                    baro_healthy = is_baro_healthy_flight(baro_pressure, baro_temp);
                    imu_healthy = is_imu_healthy_flight(ax*9.81f, ay*9.81f, az*9.81f, gx, gy, gz);

                    // FILTER UPDATE
                    float qw, qx, qy, qz;
                    orientation.updateIMU(gx, gy, gz, ax, ay, az);
                    orientation.getQuaternion(&qw, &qx, &qy, &qz);

                    float cos_tilt = std::clamp(1 - 2*(qx*qx + qy*qy), -1.0f, 1.0f);
                    float attitude_rad = acosf(cos_tilt); // same as acos(cos p * cos r) but NaN-safe
                    float vertical_accel = 2*(qx*qz - qw*qy) * ax + 2*(qy*qz + qw*qx) * ay + (1 - 2*(qx*qx + qy*qy)) * az;

                    altitude.predict(vertical_accel, attitude_rad, false);

                    // init P0
                    if (P0 == 0.0f && baro_pressure > 0.0f) {
                        P0 = baro_pressure;
                        baro_temp = 15.0f;
                        ground_temperature_k = baro_temp + 273.15f;
                    }

                    float alt_baro = compute_altitude(baro_pressure, P0);
                    altitude.update(alt_baro);

                    altitude_filtered_data.push_back(altitude.getState()[0]);
                    delta_data.push_back(altitude.getState()[0] - alt_vega);

                    RocketState rocket_state = parachute_task(altitude.getState()[1], altitude.getState()[0], altitude.getState()[2], current_ts);
                    vspeed_data.push_back(altitude.getState()[1]);

                    if (ts_data.empty() || rocket_state != last_rocket_state) {
                        std::string state_name;
                        switch(rocket_state) {
                            case RS_IDLE: state_name = "IDLE"; break;
                            case RS_BOOST: state_name = "BOOST"; break;
                            case RS_COAST: state_name = "COAST"; break;
                            case RS_DROGUE: state_name = "DROGUE"; break;
                            case RS_MAIN: state_name = "MAIN"; break;
                            case RS_TOUCHDOWN: state_name = "TOUCHDOWN"; break;
                            default: state_name = "UNKNOWN"; break;
                        }
                        state_changes.push_back({current_ts, state_name});
                        last_rocket_state = rocket_state;
                    }

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
                if (!available_ports.empty() && (size_t)selected_port_idx < available_ports.size()) {
                    current_selection = available_ports[selected_port_idx];
                }
                available_ports = get_available_ports();
                selected_port_idx = 0;
                for (int i = 0; (size_t)i < available_ports.size(); i++) {
                    if (available_ports[i] == current_selection) {
                        selected_port_idx = i;
                        break;
                    }
                }
            }

            ImGui::Checkbox("Enable Serial Output", &enable_serial);

            ImGui::BeginDisabled(!enable_serial);
            ImGui::Text("Select Serial Port (Auto-updating):");
            std::string preview_value = available_ports.empty() ? "No ports found" : available_ports[selected_port_idx];

            ImGui::PushItemWidth(250);
            if (ImGui::BeginCombo("##Serial Port", preview_value.c_str())) {
                for (int i = 0; (size_t)i < available_ports.size(); i++) {
                    const bool is_selected = (selected_port_idx == i);
                    if (ImGui::Selectable(available_ports[i].c_str(), is_selected)) {
                        selected_port_idx = i;
                    }
                    if (is_selected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::PopItemWidth();
            ImGui::EndDisabled();

            ImGui::Separator();

            if (ImGui::Button("START STREAMING AND PLOTTING", ImVec2(300, 50))) {
                if (enable_serial && !available_ports.empty()) {
                    serial_fd = init_serial(available_ports[selected_port_idx].c_str());
                    if (serial_fd == -1) {
                        std::cerr << "Failed to open serial port " << available_ports[selected_port_idx] << "\n";
                    }
                }

                if (enable_serial) {
                    std::cout << header << std::flush;
                    if (serial_fd != -1) {
                        write(serial_fd, header.c_str(), header.length());
                    }
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
                state_changes.clear();
                fault_changes.clear();
                last_rocket_state = RS_IDLE;
                current_ts = 0.0f;
                P0 = 0.0f; // Reset pressure reference
                imu_healthy = true;
                baro_healthy = true;

                file_imu.clear(); file_imu.seekg(0);
                file_baro.clear(); file_baro.seekg(0);
                file_filtered.clear(); file_filtered.seekg(0);

                std::getline(file_imu, line_imu);
                std::getline(file_baro, line_baro);
                std::getline(file_filtered, line_filtered);

                if (enable_serial) {
                    std::cout << header << std::flush;
                    if (serial_fd != -1) {
                        write(serial_fd, header.c_str(), header.length());
                    }
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

            ImGui::Spacing();
            ImGui::Text("IMU Status: ");
            ImGui::SameLine();
            if (imu_healthy) ImGui::TextColored(ImVec4(0, 1, 0, 1), "HEALTHY");
            else ImGui::TextColored(ImVec4(1, 0, 0, 1), "FAULT");

            ImGui::SameLine(250);
            ImGui::Text("BARO Status: ");
            ImGui::SameLine();
            if (baro_healthy) ImGui::TextColored(ImVec4(0, 1, 0, 1), "HEALTHY");
            else ImGui::TextColored(ImVec4(1, 0, 0, 1), "FAULT");

            ImGui::Separator();

            if (!ts_data.empty()) {
                ImVec2 plot_size = ImVec2(-1, 300);

                auto drawStateLines = [&]() {
                    for (const auto& change : state_changes) {
                        double x[1] = { change.first };
                        ImPlot::PlotInfLines(change.second.c_str(), x, 1);
                        ImPlot::Annotation(change.first, ImPlot::GetPlotLimits().Y.Min, ImVec4(1,1,1,1), ImVec2(5,-15), false, "%s", change.second.c_str());
                    }
                    for (const auto& fault : fault_changes) {
                        double x[1] = { fault.first };

                        ImPlotSpec spec;
                        spec.LineColor = ImVec4(1, 0, 0, 1); // Rosso per i fault

                        ImPlot::PlotInfLines(fault.second.c_str(), x, 1, spec);
                        ImPlot::Annotation(fault.first, ImPlot::GetPlotLimits().Y.Max, ImVec4(1,0,0,1), ImVec2(5, 15), false, "%s", fault.second.c_str());
                    }
                };

                if (ImPlot::BeginPlot("Altitude", plot_size)) {
                    ImPlot::SetupAxes("Time (s)", "Altitude AGL (m)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotLine("filteredAltitudeAGL", ts_data.data(), alt_data.data(), ts_data.size());
                    drawStateLines();
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Altitude StarFly", plot_size)) {
                    ImPlot::SetupAxes("Time (s)", "Altitude AGL (m)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotLine("filteredAltitudeAGL", ts_data.data(), altitude_filtered_data.data(), ts_data.size());
                    drawStateLines();
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("VSpeed StarFly", plot_size)) {
                    ImPlot::SetupAxes("Time (s)", "Vertical Speed (m/s)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotLine("filteredVspeed", ts_data.data(), vspeed_data.data(), ts_data.size());
                    drawStateLines();
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Delta (Starfly - CSV)", plot_size)) {
                    ImPlot::SetupAxes("Time (s)", "Delta (m)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotLine("Delta", ts_data.data(), delta_data.data(), ts_data.size());
                    drawStateLines();
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Acceleration", plot_size)) {
                    ImPlot::SetupAxes("Time (s)", "Acceleration (g)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::PlotLine("filteredAcceleration", ts_data.data(), acc_data.data(), ts_data.size());
                    drawStateLines();
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
