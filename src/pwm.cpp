#include <zephyr.h>
#include <drivers/pwm.h>
#include <devicetree.h>
#include "pwm.hpp"

// TODO FIX, just hack..
#define PWM_ARRAY DT_N_S_pwm_array_0

PWM::PWM() {
    _pwm_devs[0] = DEVICE_DT_GET(DT_PWMS_CTLR_BY_IDX(PWM_ARRAY, 0));
    _pwm_devs[1] = DEVICE_DT_GET(DT_PWMS_CTLR_BY_IDX(PWM_ARRAY, 1));
    _pwm_devs[2] = DEVICE_DT_GET(DT_PWMS_CTLR_BY_IDX(PWM_ARRAY, 2));
    _pwm_devs[3] = DEVICE_DT_GET(DT_PWMS_CTLR_BY_IDX(PWM_ARRAY, 3));

    _pwm_channels[0] = DT_PWMS_CELL_BY_IDX(PWM_ARRAY, 0, channel);
    _pwm_channels[1] = DT_PWMS_CELL_BY_IDX(PWM_ARRAY, 1, channel);
    _pwm_channels[2] = DT_PWMS_CELL_BY_IDX(PWM_ARRAY, 2, channel);
    _pwm_channels[3] = DT_PWMS_CELL_BY_IDX(PWM_ARRAY, 3, channel);
};

bool PWM::is_ready() const {
    bool ret = true;
    return ret;
}

void PWM::write(const float (&outputs)[4]) {
    if (!is_ready()) {
        return;
    }

    for (int i=0; i<4; i++) {
        uint32_t pulse = _min_duty + static_cast<uint32_t>( (_max_duty - _min_duty) * outputs[i]);
        int ret = pwm_pin_set_usec(_pwm_devs[i], 1, _period, pulse, 0);

        if (ret < 0) {
            printk("EROORORORO %d\n", ret);
            break;
        }
    }

#ifdef CONFIG_BOARD_STM32F3_DISCO
    // Put correct configuration into stm32f303 register..
    volatile int * volatile reg = (volatile int*)0x40012C20;
    *reg = 0x1444;
#endif
}
