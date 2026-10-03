#include <iostream>
#include <cmath>

//check a terra
//todo: aggiusta i threshold
bool is_imu_healthy_ground(float ax, float ay, float az, float gx, float gy, float gz, int& variance_checks) {

    static int count = 0;
    static constexpr int max_count = 10; 

    //controllo threshold acc e gyro

    constexpr float gyro_threshold = 5.0f; 
    bool gyro_thr_ok = std::abs(gx) < gyro_threshold && std::abs(gy) < gyro_threshold && std::abs(gz) < gyro_threshold;



    //controllo acc = 1g
    float gravity = std::sqrt(ax *ax + ay*ay + az*az);
    float gravity_threshold = 0.1f;
    bool gravity_ok = std::abs(gravity - 1.0f) < gravity_threshold;
    


    //controllo varianza 
    static constexpr int samples = 10;
    static constexpr float acc_variance_threshold = (10 * 5.4f * 0.001f) * (10 *5.4f * 0.001f);// 5.4 mg rumore minimo
    static constexpr float gyro_variance_threshold = (10 * 75.0f * 0.001f) * (10 * 75.0f * 0.001f); //75 mdps rumore minimo

    static float ax_samples[samples];
    static float ay_samples[samples];
    static float az_samples[samples];
    
    static float gx_samples[samples];
    static float gy_samples[samples];
    static float gz_samples[samples];

    static int sample_index = 0;


    ax_samples[sample_index] = ax;
    ay_samples[sample_index] = ay;
    az_samples[sample_index] = az;

    gx_samples[sample_index] = gx;
    gy_samples[sample_index] = gy;
    gz_samples[sample_index] = gz;

    sample_index = (sample_index + 1) % samples;

    static bool variance_ok = true;

    if (sample_index == 0) {
        variance_checks++; // Increment counter
        
        float ax_mean = 0, ay_mean = 0, az_mean = 0;
        float gx_mean = 0, gy_mean = 0, gz_mean = 0;

        // Calcolo di tutte le medie in un solo ciclo (ottimizzato!)
        for (int i = 0; i < samples; i++) {
            ax_mean += ax_samples[i];
            ay_mean += ay_samples[i];
            az_mean += az_samples[i];
            gx_mean += gx_samples[i];
            gy_mean += gy_samples[i];
            gz_mean += gz_samples[i];
        }
        ax_mean /= samples; ay_mean /= samples; az_mean /= samples;
        gx_mean /= samples; gy_mean /= samples; gz_mean /= samples;

        float ax_variance = 0, ay_variance = 0, az_variance = 0;
        float gx_variance = 0, gy_variance = 0, gz_variance = 0;

        // Calcolo di tutte le varianze in un solo ciclo
        for (int i = 0; i < samples; i++) {
            ax_variance += (ax_samples[i] - ax_mean) * (ax_samples[i] - ax_mean);
            ay_variance += (ay_samples[i] - ay_mean) * (ay_samples[i] - ay_mean);
            az_variance += (az_samples[i] - az_mean) * (az_samples[i] - az_mean);
            gx_variance += (gx_samples[i] - gx_mean) * (gx_samples[i] - gx_mean);
            gy_variance += (gy_samples[i] - gy_mean) * (gy_samples[i] - gy_mean);
            gz_variance += (gz_samples[i] - gz_mean) * (gz_samples[i] - gz_mean);
        }
        ax_variance /= samples; ay_variance /= samples; az_variance /= samples;
        gx_variance /= samples; gy_variance /= samples; gz_variance /= samples;

        // Controllo che NESSUN asse sia congelato (varianza troppo piccola) 
        // e che NESSUN asse sia troppo rumoroso (varianza sopra il threshold)
        variance_ok = 
            (ax_variance < acc_variance_threshold && ax_variance > std::numeric_limits<float>::epsilon()) &&
            (ay_variance < acc_variance_threshold && ay_variance > std::numeric_limits<float>::epsilon()) &&
            (az_variance < acc_variance_threshold && az_variance > std::numeric_limits<float>::epsilon()) &&
            (gx_variance < gyro_variance_threshold && gx_variance > std::numeric_limits<float>::epsilon()) &&
            (gy_variance < gyro_variance_threshold && gy_variance > std::numeric_limits<float>::epsilon()) &&
            (gz_variance < gyro_variance_threshold && gz_variance > std::numeric_limits<float>::epsilon());
    }


    if (!gyro_thr_ok || !gravity_ok || !variance_ok) {
        count++;
    }
    else count = 0;

    return count < max_count;
}


bool is_baro_healthy_ground(float pressure, float temperature, int& variance_checks) {
    
    static int count = 0;
    static constexpr int max_count = 10; 


    bool temp_ok = temperature < 70.0f && temperature > 10.0f;
    bool pressure_ok = pressure < 104000.0f && pressure > 94000.0f;


    static bool variance_ok = true;

    static constexpr float variance_thr = 100.0f * 2.4f * 2.4f; 
    static constexpr int samples = 10;
    static float press_samples[samples];
    static int sample_index = 0;

    press_samples[sample_index] = pressure;

    sample_index = (sample_index + 1) % samples;

    if (sample_index == 0){
        variance_checks++; // Increment counter

        float press_mean = 0;

        for (int i = 0; i < samples; i++){
            press_mean += press_samples[i];
        }
        press_mean /= samples;

        float press_variance = 0;
        for (int i = 0; i < samples; i++){
            press_variance += (press_samples[i] - press_mean) * (press_samples[i] - press_mean);
        }

        press_variance /= samples;

        variance_ok = press_variance < variance_thr && press_variance > std::numeric_limits<float>::epsilon();

    } 



    if (!temp_ok || !pressure_ok || !variance_ok) count++;
    else count = 0;

    return count < max_count;
}


