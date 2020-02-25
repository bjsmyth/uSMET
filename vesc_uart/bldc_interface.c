/*
	Copyright 2016-2018 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * bldc_interface.c
 *
 * Compatible Firmware Versions
 * 3.39
 * 3.40
 *
 */

#include "bldc_interface.h"
#include "buffer.h"
#include <string.h>
#include "config.h"

// Private variables
static unsigned char send_buffer[VESC_UART_COUNT][512];

// Private variables for received data
static mc_values values[VESC_UART_COUNT];
static float rotor_pos[VESC_UART_COUNT];
static float dec_ppm[VESC_UART_COUNT];
static float dec_ppm_len[VESC_UART_COUNT];
static float dec_adc[VESC_UART_COUNT];
static float dec_adc_voltage[VESC_UART_COUNT];
static float dec_chuk[VESC_UART_COUNT];

// Private functions
void send_packet_no_fwd(unsigned int uart_index, unsigned char *data, unsigned int len);

// Function pointers
static void(*send_func)(unsigned int uart_index, unsigned char *data, unsigned int len) = 0;
static void(*forward_func)(unsigned int uart_index, unsigned char *data, unsigned int len) = 0;

// Function pointers for received data
static void(*rx_value_func)(unsigned int uart_index, mc_values *values) = 0;
static void(*rx_printf_func)(unsigned int uart_index, char *str) = 0;
static void(*rx_rotor_pos_func)(unsigned int uart_index, float pos) = 0;
static void(*rx_dec_ppm_func)(unsigned int uart_index, float val, float ms) = 0;
static void(*rx_dec_adc_func)(unsigned int uart_index, float val, float voltage) = 0;
static void(*rx_dec_chuk_func)(unsigned int uart_index, float val) = 0;
static void(*motor_control_set_func)(unsigned int uart_index, motor_control_mode mode, float value) = 0;
static void(*values_requested_func)(unsigned int uart_index) = 0;

void bldc_interface_init(void(*func)(unsigned int uart_index, unsigned char *data, unsigned int len)) {
	send_func = func;
}

void bldc_interface_set_forward_func(void(*func)(unsigned int uart_index, unsigned char *data, unsigned int len)) {
	forward_func= func;
}

/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void bldc_interface_send_packet(unsigned int uart_index, unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(uart_index, data, len);
	}
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 */
void bldc_interface_process_packet(unsigned int uart_index, unsigned char *data, unsigned int len) {
	if (!len) {
		return;
	}

	if (forward_func) {
		forward_func(uart_index, data, len);
		return;
	}

	int32_t ind = 0;
	unsigned char id = data[0];
	data++;
	len--;

	switch (id) {
	case COMM_FW_VERSION:
		break;

	case COMM_ERASE_NEW_APP:
	case COMM_WRITE_NEW_APP_DATA:
		// TODO
		break;

	case COMM_GET_VALUES:
		ind = 0;
		values[uart_index].temp_mos = buffer_get_float16(data, 1e1, &ind);
		values[uart_index].temp_motor = buffer_get_float16(data, 1e1, &ind);
		values[uart_index].current_motor = buffer_get_float32(data, 1e2, &ind);
		values[uart_index].current_in = buffer_get_float32(data, 1e2, &ind);
		values[uart_index].id = buffer_get_float32(data, 1e2, &ind);
		values[uart_index].iq = buffer_get_float32(data, 1e2, &ind);
		values[uart_index].duty_now = buffer_get_float16(data, 1e3, &ind);
		values[uart_index].rpm = buffer_get_float32(data, 1e0, &ind);
		values[uart_index].v_in = buffer_get_float16(data, 1e1, &ind);
		values[uart_index].amp_hours = buffer_get_float32(data, 1e4, &ind);
		values[uart_index].amp_hours_charged = buffer_get_float32(data, 1e4, &ind);
		values[uart_index].watt_hours = buffer_get_float32(data, 1e4, &ind);
		values[uart_index].watt_hours_charged = buffer_get_float32(data, 1e4, &ind);
		values[uart_index].tachometer = buffer_get_int32(data, &ind);
		values[uart_index].tachometer_abs = buffer_get_int32(data, &ind);
		values[uart_index].fault_code = (mc_fault_code)data[ind++];

		if (ind < (int)len) {
			values[uart_index].pid_pos = buffer_get_float32(data, 1e6, &ind);
		} else {
			values[uart_index].pid_pos = 0.0;
		}

		if (ind < (int)len) {
			values[uart_index].vesc_id = data[ind++];
		} else {
			values[uart_index].vesc_id = 255;
		}

		if (rx_value_func) {
			rx_value_func(uart_index, &values[uart_index]);
		}
		break;

	case COMM_PRINT:
		if (rx_printf_func) {
			data[len] = '\0';
			rx_printf_func(uart_index, (char*)data);
		}
		break;

	case COMM_SAMPLE_PRINT:
		// TODO
		break;

	case COMM_ROTOR_POSITION:
		ind = 0;
		rotor_pos[uart_index] = buffer_get_float32(data, 100000.0, &ind);

		if (rx_rotor_pos_func) {
			rx_rotor_pos_func(uart_index, rotor_pos[uart_index]);
		}
		break;

	case COMM_EXPERIMENT_SAMPLE:
		// TODO
		break;

	case COMM_GET_MCCONF:
	case COMM_GET_MCCONF_DEFAULT:
		break;

	case COMM_GET_APPCONF:
	case COMM_GET_APPCONF_DEFAULT:
		break;

	case COMM_DETECT_MOTOR_PARAM:
		break;

	case COMM_DETECT_MOTOR_R_L: {
		// TODO!
	} break;

	case COMM_DETECT_MOTOR_FLUX_LINKAGE: {
		// TODO!
	} break;

	case COMM_DETECT_ENCODER: {
		// TODO!
	} break;

	case COMM_DETECT_HALL_FOC: {
		// TODO!
	} break;

	case COMM_GET_DECODED_PPM:
		ind = 0;
		dec_ppm[uart_index] = buffer_get_float32(data, 1000000.0, &ind);
		dec_ppm_len[uart_index] = buffer_get_float32(data, 1000000.0, &ind);

		if (rx_dec_ppm_func) {
			rx_dec_ppm_func(uart_index, dec_ppm[uart_index], dec_ppm_len[uart_index]);
		}
		break;

	case COMM_GET_DECODED_ADC:
		ind = 0;
		dec_adc[uart_index] = buffer_get_float32(data, 1000000.0, &ind);
		dec_adc_voltage[uart_index] = buffer_get_float32(data, 1000000.0, &ind);
		// TODO for adc2

		if (rx_dec_adc_func) {
			rx_dec_adc_func(uart_index, dec_adc[uart_index], dec_adc_voltage[uart_index]);
		}
		break;

	case COMM_GET_DECODED_CHUK:
		ind = 0;
		dec_chuk[uart_index] = buffer_get_float32(data, 1000000.0, &ind);

		if (rx_dec_chuk_func) {
			rx_dec_chuk_func(uart_index, dec_chuk[uart_index]);
		}
		break;

	case COMM_SET_MCCONF:
		// This is a confirmation that the new mcconf is received.
		break;

	case COMM_SET_APPCONF:
		// This is a confirmation that the new appconf is received.}
		break;

	default:
		break;
	}
}

/**
 * Function pointer setters. When data that is requested with the get functions
 * is received, the corresponding function pointer will be called with the
 * received data.
 *
 * @param func
 * A function to be called when the corresponding data is received.
 */

void bldc_interface_set_rx_value_func(void(*func)(unsigned int uart_index, mc_values *values)) {
	rx_value_func = func;
}

void bldc_interface_set_rx_printf_func(void(*func)(unsigned int uart_index, char *str)) {
	rx_printf_func= func;
}

void bldc_interface_set_rx_rotor_pos_func(void(*func)(unsigned int uart_index, float pos)) {
	rx_rotor_pos_func = func;
}

void bldc_interface_set_rx_dec_ppm_func(void(*func)(unsigned int uart_index, float val, float ms)) {
	rx_dec_ppm_func = func;
}

void bldc_interface_set_rx_dec_adc_func(void(*func)(unsigned int uart_index, float val, float voltage)) {
	rx_dec_adc_func = func;
}

void bldc_interface_set_rx_dec_chuk_func(void(*func)(unsigned int uart_index, float val)) {
	rx_dec_chuk_func = func;
}

void bldc_interface_set_sim_control_function(void(*func)(unsigned int uart_index, motor_control_mode mode, float value)) {
	motor_control_set_func = func;
}

void bldc_interface_set_sim_values_func(void(*func)(unsigned int uart_index)) {
	values_requested_func = func;
}

// Setters
void bldc_interface_terminal_cmd(unsigned int uart_index, char* cmd) {
	int len = strlen(cmd);
	send_buffer[uart_index][0] = COMM_TERMINAL_CMD;
	memcpy(send_buffer[uart_index] + 1, cmd, len);
	send_packet_no_fwd(uart_index, send_buffer[uart_index], len + 1);
}

void bldc_interface_set_duty_cycle(unsigned int uart_index, float dutyCycle) {
	if (motor_control_set_func) {
		motor_control_set_func(uart_index, MOTOR_CONTROL_DUTY, dutyCycle);
		return;
	}
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_SET_DUTY;
	buffer_append_float32(send_buffer[uart_index], dutyCycle, 100000.0, &send_index);
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void bldc_interface_set_current(unsigned int uart_index, float current) {
	if (motor_control_set_func) {
		motor_control_set_func(uart_index, MOTOR_CONTROL_CURRENT, current);
		return;
	}
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_SET_CURRENT;
	buffer_append_float32(send_buffer[uart_index], current, 1000.0, &send_index);
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void bldc_interface_set_current_brake(unsigned int uart_index, float current) {
	if (motor_control_set_func) {
		motor_control_set_func(uart_index, MOTOR_CONTROL_CURRENT_BRAKE, current);
		return;
	}
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_float32(send_buffer[uart_index], current, 1000.0, &send_index);
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void bldc_interface_set_rpm(unsigned int uart_index, int rpm) {
	if (motor_control_set_func) {
		motor_control_set_func(uart_index, MOTOR_CONTROL_RPM, rpm);
		return;
	}
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_SET_RPM;
	buffer_append_int32(send_buffer[uart_index], rpm, &send_index);
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void bldc_interface_set_rpm_true(unsigned int uart_index, int rpm) {
    bldc_interface_set_rpm(uart_index, rpm * MOTOR_POLE_PAIRS);
}

void bldc_interface_set_pos(unsigned int uart_index, float pos) {
	if (motor_control_set_func) {
		motor_control_set_func(uart_index, MOTOR_CONTROL_POS, pos);
		return;
	}
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_SET_POS;
	buffer_append_float32(send_buffer[uart_index], pos, 1000000.0, &send_index);
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void bldc_interface_set_handbrake(unsigned int uart_index, float current) {
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_SET_HANDBRAKE;
	buffer_append_float32(send_buffer[uart_index], current, 1e3, &send_index);
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void bldc_interface_set_servo_pos(unsigned int uart_index, float pos) {
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_SET_SERVO_POS;
	buffer_append_float16(send_buffer[uart_index], pos, 1000.0, &send_index);
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

// Getters
void bldc_interface_get_fw_version(unsigned int uart_index) {
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_FW_VERSION;
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void bldc_interface_get_values(unsigned int uart_index) {
	if (values_requested_func) {
		values_requested_func(uart_index);
		return;
	}
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_GET_VALUES;
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void bldc_interface_get_decoded_ppm(unsigned int uart_index) {
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_GET_DECODED_PPM;
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void bldc_interface_get_decoded_adc(unsigned int uart_index) {
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_GET_DECODED_ADC;
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void bldc_interface_get_decoded_chuk(unsigned int uart_index) {
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_GET_DECODED_CHUK;
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

// Other functions
void bldc_interface_detect_motor_param(unsigned int uart_index, float current, float min_rpm, float low_duty) {
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_DETECT_MOTOR_PARAM;
	buffer_append_float32(send_buffer[uart_index], current, 1000.0, &send_index);
	buffer_append_float32(send_buffer[uart_index], min_rpm, 1000.0, &send_index);
	buffer_append_float32(send_buffer[uart_index], low_duty, 1000.0, &send_index);
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void bldc_interface_reboot(unsigned int uart_index) {
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_REBOOT;
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void bldc_interface_send_alive(unsigned int uart_index) {
	int32_t send_index = 0;
	send_buffer[uart_index][send_index++] = COMM_ALIVE;
	send_packet_no_fwd(uart_index, send_buffer[uart_index], send_index);
}

void send_values_to_receiver(unsigned int uart_index, mc_values *values) {
	if (rx_value_func) {
		rx_value_func(uart_index, values);
	}
}

// Helpers
const char* bldc_interface_fault_to_string(mc_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE";
	case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE";
	case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE";
	case FAULT_CODE_DRV: return "FAULT_CODE_DRV";
	case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT";
	case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET";
	case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR";
	default: return "Unknown fault";
	}
}

// Private functions
void send_packet_no_fwd(unsigned int uart_index, unsigned char *data, unsigned int len) {
	if (!forward_func) {
		bldc_interface_send_packet(uart_index, data, len);
	}
}
