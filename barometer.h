#pragma once
#include <cstdint>
#include <cmath>


struct BaroData {
	uint32_t timestamp;
	float temperature;
	float pressure;
	float altitude;
	int error_status;
};



float ground_pressure_mbar_1 = 0.0; // memorizzo pressione misurata sulla rampa di lancio
float ground_pressure_mbar_2 = 0.0; // memorizzo pressione misurata sulla rampa di lancio
float ground_temperature_k = 0.0;




// https://www.mide.com/air-pressure-at-altitude-calculator
float compute_altitude(float air_pressure, float ground_pressure)
{
	// FIXME: Lb and M change with weather conditions
	const float Lb = -0.0065; // standard temperature lapse rate [K/m]
	const float M = 0.0289644; // Molar mass of air [kg/mol]
	const float R = 8.31432;  // Universal gas constant [N*m/mol*K]
	const float g0 = 9.80665; // Gravitational constant [m/s^2]

	float x = ground_temperature_k / Lb;
	float p = air_pressure / ground_pressure;
	float e = -(R*Lb)/(g0*M);

	return x*(pow(p, e) - 1);
}

