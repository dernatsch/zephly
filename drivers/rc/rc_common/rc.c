/*
 * Copyright (c) 2022 Jonathan Hahn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <custom_drivers/rc.h>
#include <settings/settings.h>
#include <shell/shell.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* default settings */
#define RC_EXPO_ROLL_DEFAULT 0.0
#define RC_EXPO_PITCH_DEFAULT 0.0
#define RC_EXPO_YAW_DEFAULT 0.0

#define RC_SUPEREXPO_ROLL_DEFAULT 0.0
#define RC_SUPEREXPO_PITCH_DEFAULT 0.0
#define RC_SUPEREXPO_YAW_DEFAULT 0.0

enum RC_SETTINGS {
    RC_EXPO_ROLL = 0,
    RC_EXPO_PITCH,
    RC_EXPO_YAW,
    RC_SUPEREXPO_ROLL,
    RC_SUPEREXPO_PITCH,
    RC_SUPEREXPO_YAW
};

/* settings variables */
static float rc_settings[6] = {
    RC_EXPO_ROLL_DEFAULT,
    RC_EXPO_PITCH_DEFAULT,
    RC_EXPO_YAW_DEFAULT,
    RC_SUPEREXPO_ROLL_DEFAULT,
    RC_SUPEREXPO_PITCH_DEFAULT,
    RC_SUPEREXPO_YAW_DEFAULT
};

const char *rc_setting_names[6] = {
    "expo_roll",
    "expo_pitch",
    "expo_yaw",
    "superexpo_roll",
    "superexpo_pitch",
    "superexpo_yaw"
};

#if IS_ENABLED(CONFIG_SHELL)
K_MUTEX_DEFINE(rc_settings_mutex);
#endif

static int rc_settings_set(const char *name, float value) {
    float *settings_value = NULL;

    for (int i=0; i<6; i++) {
        if (strcmp(name, rc_setting_names[i]) == 0) {
            settings_value = &rc_settings[i];
        }
    } 
    
    if (settings_value == NULL) {
        printk("Invalid parameter name.\n");
        return -EINVAL;
    }

#if IS_ENABLED(CONFIG_SHELL)
    if (k_mutex_lock(&rc_settings_mutex, K_MSEC(100)) != 0) {
        printk("Could not aquire rc_settings lock.\n");
        return -ENOLCK;
    }
#endif

    *settings_value = value;

#if IS_ENABLED(CONFIG_SHELL)
    k_mutex_unlock(&rc_settings_mutex);
#endif

    return 0;
}

#if IS_ENABLED(CONFIG_SHELL)

static int rc_cmd_settings_set(const struct shell *sh, size_t argc, char **argv) {
    if (argc != 3) {
        return -EINVAL;
    }

    const char *param_name = argv[1];
    const float tmp = atof(argv[2]);

    int rc = rc_settings_set(param_name, tmp);

    if (rc != 0) {
        printk("Could not set parameter.\n");
        return -EINVAL;
    }

    return 0;
}

static int rc_cmd_show_settings(const struct shell *sh, size_t argc, char **argv) {
    if (k_mutex_lock(&rc_settings_mutex, K_MSEC(100)) != 0) {
        printk("Could not aquire rc_settings lock.\n");
        return -ENOLCK;
    }

    for (int i=0; i<6; i++) {
        printf("%s: %f\n", rc_setting_names[i], rc_settings[i]);
    }

    k_mutex_unlock(&rc_settings_mutex);
}

SHELL_CMD_ARG_REGISTER(rc_set, NULL, "Read MPU6000 register", rc_cmd_settings_set, 3, 0);
SHELL_CMD_REGISTER(rc_show, NULL, "Print MPU6000 data", rc_cmd_show_settings);

#endif

static inline float clip(float in, float min_out, float max_out) {
    if (in < min_out) {
            return min_out; 
    } else if (in > max_out) { 
        return max_out; 
    } else {
        return in;
    }
}

float apply_superexpo(float val, float e, float g) {
    val = clip(val, -0.999, 0.999),
    e = clip(e, 0.0, 0.99);
    g = clip(g, 0.0, 1.0);
    float expo = (1.0f - e) * val + e * val * val * val;
    float superexpo = expo * (1.0f - g) / (1 - fabs(val) * g);

    return superexpo;
}

void rc_update_expo(const struct device *dev, struct Command *rc_in) {
    rc_update(dev, rc_in);

    rc_in->roll = apply_superexpo(rc_in->roll, rc_settings[RC_EXPO_ROLL], rc_settings[RC_SUPEREXPO_ROLL]);
    rc_in->pitch = apply_superexpo(rc_in->pitch, rc_settings[RC_EXPO_PITCH], rc_settings[RC_SUPEREXPO_PITCH]);
    rc_in->yaw = apply_superexpo(rc_in->yaw, rc_settings[RC_EXPO_YAW], rc_settings[RC_SUPEREXPO_YAW]);
    rc_in->thrust = clip(rc_in->thrust, 0.0f, 1.0f);
}
