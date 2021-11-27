#include "sensors.hpp"
#include "kernel.h"
#include <device.h>
#include <stdio.h>
#include <zephyr.h>
#include <math.h>

#include <attitude_estimation.hpp>
#include "pwm.hpp"
#include "rc/rc.hpp"
#include <msgs.hpp>
//#include <mixer.hpp>

constexpr double DEG2RAD = 0.017453292519943295;

using namespace AttitudeEstimation;

float outputs[] = {0.5f, 0.5f, 0.5f, 0.5f};
Command rc_input;

void report_state(const Quaternion<double> &q, const Vector<double, 3> &w, const Vector<double, 3> &a);
void cmd2outputs(Command cmd, float *outputs);

static float B_inv[4][4] = {
    { 1.f,  1.f, -1.f, 1.f},
    { 1.f, -1.f,  1.f, 1.f},
    {-1.f, -1.f, -1.f, 1.f},
    {-1.f,  1.f,  1.f, 1.f}
};

void simple_mix2(Command cmd, float (&outputs)[4]) {

    float inputs[4] = {cmd.roll, cmd.pitch, cmd.yaw, cmd.throttle};

    for (int i=0; i<4; i++) {
        // reset outputs
        outputs[i] = 0.f;

        for (int j=0; j<4; j++) {
            // calculate new mixed values
            outputs[i] += B_inv[i][j] * inputs[j];
        }

        // clamp output
        if (outputs[i] < 0.f) { outputs[i] = 0.f; }
        if (outputs[i] > 1.f) { outputs[i] = 1.f; }
    }
}

int main() {
	auto &pwm = PWM::init();

	printk("TEST\n");

	if(rc_init() < 0) {
		printk("RC Initialization failed!\n");
		return -1;
	}
	printk("TEST2.1\n");
	if (!initialize_sensors() || !pwm.is_ready()) {
		printk("Initialization failed!");
		return -1;
	}
	printk("TEST2\n");
	pwm.write(outputs);
	
	printk("TEST2\n");
	
	/* initial attitude */
	Quaternion<double> q = {1.0, 0.0, 0.0, 0.0};

	rc_run();
	
	printk("TEST3\n");

	while (true) {

		rc_input = rc_get();
		//rc_print(rc_input);

		simple_mix2(rc_input, outputs);

		pwm.write(outputs);
		printf("o:\t%f\t%f\t%f\t%f\n", outputs[0], outputs[1], outputs[2], outputs[3]);

		if (!update_measurements()) {
			/* Something went wrong, try again */
			printk("Update failed!");
			continue;
		}

		propagate_attitude<double>(q, get_rotation_speed(), get_delta_t());

		//report_state(q, get_rotation_speed(), get_accelerations());

		k_sleep(K_MSEC(50));
	}

	return 0;
}

void report_state(const Quaternion<double> &q, const Vector<double, 3> &w, const Vector<double, 3> &a) {
	printf("%f %f %f %f", q.w, q.x, q.y, q.z);
	printf(" %u %f %f %f", k_uptime_get_32(), w.data[0], w.data[1], w.data[2]);
	printf(" %f %f %f \n", a.data[0], a.data[1], a.data[2]);
}

void cmd2outputs(Command cmd, float *outputs) {
	outputs[0] = cmd.pitch;
	outputs[1] = cmd.roll;
	outputs[2] = cmd.throttle;
	outputs[3] = cmd.yaw;
}
