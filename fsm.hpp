#include <iostream>
#include <cmath>

// funzione quasi uguale a quella originale
// togliere i commenti
// sostituire millis() con time_s * 1000
// sistemare parametri
// reinserisci rocketstate dentro
//togli static

enum RocketState {
		RS_IDLE,      // Idle state, on ramp
		RS_BOOST,     // Motor burning, ascending
		RS_COAST,     // Motor burnt out, still ascending
		RS_DROGUE,    // Drogue deployed, falling
		RS_MAIN,      // Main parachute deployed, falling
		RS_TOUCHDOWN, // On ground
	};

RocketState parachute_task(float _z_speed, float _z_alt, float _z_acc, float time_s)
{
	// self->last_wake = xTaskGetTickCount();

	static RocketState state = RS_IDLE;



	static float max_alt = 0.0f;
	max_alt = std::max(max_alt, _z_alt);


	/* TODO:
	 * - Change unit names m/s to MPS
	 * - Change timers from all being referenced to ignition to being referenced to the last state change
	 * - Remove touchdown timer
	 * - Increase accelleration threshold to 3.0g min
	 * - Decrease altitude threshold for main deployment to 400m
	 * - Make this a configuration file or header
	 */

	#define PARACHUTE_TASK_HZ 10

	#define Z_ACC_BOOST_THRESHOLD_G 3
	#define Z_SPEED_BOOST_THRESHOLD_MS 25.0
//	#define Z_ALT_BOOST_THRESHOLD_M 100.0

//	#define Z_ALT_COAST_THRESHOLD_M 750.0
//	#define MOTOR_BURNOUT_MS 4400

	#define Z_SPEED_APOGEE_THRESHOLD_MS -0.01
//	#define Z_ALT_APOGEE_THRESHOLD_M 2950.0
//	#define MAX_TIME_TO_APOGEE_MS 28000

	#define MIN_TIME_TO_1500M_MS 8540

	#define Z_ALT_MAIN_DEPLOYMENT_M 400.0
//	#define MAX_TIME_TO_MAIN_DEPLOYMENT_MS 110000

	#define Z_ALT_TOUCHDOWN_M 10.0
	#define Z_SPEED_STATIONARY_MS 0.0
//	#define MAX_TIME_TO_TOUCHDOWN 200000

	#define BOOST_DETECTION_SAMPLE_COUNT 10
	#define BURNOUT_DETECTION_SAMPLE_COUNT 10
	#define APOGEE_DETECTION_SAMPLE_COUNT 10
	#define MAIN_DETECTION_SAMPLE_COUNT 10
	#define TOUCHDOWN_DETECTION_SAMPLE_COUNT 10

	#define PIN_EJECTION_A  PINT6_LS
	#define PIN_EJECTION_C  PINT4_LS
	#define PIN_MAIN_CUTTER PINT2_LS

	#define CUTTERS_ON_TIME_MS 2000


	// Interface PINs setup
	// pinMode(PIN_EJECTION_A, OUTPUT);
	// digitalWrite(PIN_EJECTION_A, 0);
	// pinMode(PIN_EJECTION_C, OUTPUT);
	// digitalWrite(PIN_EJECTION_C, 0);
	// pinMode(PIN_MAIN_CUTTER, OUTPUT);
	// digitalWrite(PIN_MAIN_CUTTER, 0);


	// Sensor data each loop
	static float z_speed = 0;
	static float z_alt = 0;
	static float z_acc = 0;

	// Milliseconds since detected motor ignition
	static int64_t ms_since_ignition = 0;
	// Time of detected motor ignition
	static int64_t ms_ignition = 0;

	// Number of consecutive samples that met the state change condition
	static int sample_count = 0;

	static bool ejection_active = false;
	static int64_t ejection_fire_time = 0;

	static bool cutter_active = false;
	static int64_t cutter_fire_time = 0;


	// while (true) {
		// ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000/PARACHUTE_TASK_HZ));

		// Get the sensor data, if there is no new sensor data the old sample is
		// used
		// LogMessage msg;
		// while (xQueueReceive(parachute_msg_queue, &msg, 0) == pdTRUE) {
			// if (msg.type == T_ALT_SPEED && msg.payload_type == P_FVEC2) {
				z_alt = _z_alt;// msg.payload.fv2.x;
				z_speed = _z_speed;// msg.payload.fv2.y;
			// } else if (msg.type == T_ACCELLERATION && msg.payload_type == P_FVEC3) {
				static constexpr float alpha_acc = 0.1f;
				z_acc = alpha_acc * _z_acc + (1 - alpha_acc) * z_acc;// msg.payload.fv3.z;
			// }
		// }

		// FIXME: will this always work?
		ms_since_ignition = time_s * 1000 - ms_ignition;//millis() - ms_ignition;

		// Non-blocking pyro pin timeout handling — runs every pass regardless
		// of state so it can't stall queue draining
		if (ejection_active && (time_s * 1000 - ejection_fire_time) >= CUTTERS_ON_TIME_MS) {
			// pinMode(PIN_EJECTION_A, OUTPUT);
			// digitalWrite(PIN_EJECTION_A, 0);
			// pinMode(PIN_EJECTION_C, OUTPUT);
			// digitalWrite(PIN_EJECTION_C, 0);
			ejection_active = false;
		}
		if (cutter_active && (time_s * 1000 - cutter_fire_time) >= CUTTERS_ON_TIME_MS) {
			// pinMode(PIN_MAIN_CUTTER, OUTPUT);
			// digitalWrite(PIN_MAIN_CUTTER, 0);
			cutter_active = false;
		}

		switch (state) {
		case RS_IDLE:
			// Detect motor ignition
			if (z_acc >= Z_ACC_BOOST_THRESHOLD_G && z_speed >= Z_SPEED_BOOST_THRESHOLD_MS) {
				sample_count++;
			} else {
				sample_count = 0;
			}

			if (sample_count >= BOOST_DETECTION_SAMPLE_COUNT) {
				ms_ignition = time_s * 1000;//millis();
				state = RS_BOOST;
				sample_count = 0;
			}

			//log(S_PARA, T_SYSLOG, "State: RS_IDLE");
			break;

		case RS_BOOST:
			// Detect motor burnout
			if (z_acc < 0) {
				sample_count++;
			} else {
				sample_count = 0;
			}

			if (sample_count >= BURNOUT_DETECTION_SAMPLE_COUNT) {
				state = RS_COAST;
				sample_count = 0;
			}

			//log(S_PARA, T_SYSLOG, "State: RS_BOOST");
			break;

		case RS_COAST:
			if (ms_since_ignition >= MIN_TIME_TO_1500M_MS) {
				// TODO: control aibrakes
			}

			// Detect apogee
			if (z_speed <= Z_SPEED_APOGEE_THRESHOLD_MS) {
				sample_count++;
			} else {
				sample_count = 0;
			}

			if (sample_count >= APOGEE_DETECTION_SAMPLE_COUNT) {
				// Activate recovery A and C; pins are cleared later,
				// non-blockingly, by the timeout check above
				// analogWrite(PIN_EJECTION_A, 256/2);
				// digitalWrite(PIN_EJECTION_C, 1);
				ejection_active = true;
				ejection_fire_time = time_s * 1000;//millis();

				state = RS_DROGUE;
				sample_count = 0;
			}

			//log(S_PARA, T_SYSLOG, "State: RS_COAST");
			break;

		case RS_DROGUE:
			// TODO: retract airbrakes

			// Detect main parachute deployment
			if (z_alt <= Z_ALT_MAIN_DEPLOYMENT_M) {
				sample_count++;
			} else {
				sample_count = 0;
			}

			if (sample_count >= MAIN_DETECTION_SAMPLE_COUNT) {
				// Cut main parachute; pin cleared later, non-blockingly
				// analogWrite(PIN_MAIN_CUTTER, 256/2);
				cutter_active = true;
				cutter_fire_time = time_s * 1000;//millis();

				state = RS_MAIN;
				sample_count = 0;
			}

			//log(S_PARA, T_SYSLOG, "State: RS_DROGUE");
			break;

		case RS_MAIN:
			// Detect touchdown
			if (z_alt <= Z_ALT_TOUCHDOWN_M || fabs(z_speed) <= Z_SPEED_STATIONARY_MS) {
				sample_count++;
			} else {
				sample_count = 0;
			}

			if (sample_count >= TOUCHDOWN_DETECTION_SAMPLE_COUNT) {
				state = RS_TOUCHDOWN;
				sample_count = 0;
			}
			//log(S_PARA, T_SYSLOG, "State: RS_MAIN");
			break;

		case RS_TOUCHDOWN:
			//log(S_PARA, T_SYSLOG, "State: RS_TOUCHDOWN");
			break;

		default:
			//log(S_PARA, T_SYSLOG, "[ERR]: Unknown rocket state");
			break;
		}

		// log rocket state for telemetry and debugging
		// log(S_PARA, T_SYSLOG, (int)state);
	// }
    return state;
}
