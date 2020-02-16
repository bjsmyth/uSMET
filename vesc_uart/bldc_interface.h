/*
	Copyright 2016-2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef BLDC_INTERFACE_H_
#define BLDC_INTERFACE_H_

#include "datatypes.h"

// interface functions
void bldc_interface_init(void(*func)(unsigned int uart_index, unsigned char *data, unsigned int len));
void bldc_interface_set_forward_func(void(*func)(unsigned int uart_index, unsigned char *data, unsigned int len));
void bldc_interface_send_packet(unsigned int uart_index, unsigned char *data, unsigned int len);
void bldc_interface_process_packet(unsigned int uart_index, unsigned char *data, unsigned int len);

// Function pointer setters
void bldc_interface_set_rx_value_func(void(*func)(unsigned int uart_index, mc_values *values));
void bldc_interface_set_rx_printf_func(void(*func)(unsigned int uart_index, char *str));
void bldc_interface_set_rx_fw_func(void(*func)(unsigned int uart_index, int major, int minor));
void bldc_interface_set_rx_rotor_pos_func(void(*func)(unsigned int uart_index, float pos));
void bldc_interface_set_rx_dec_ppm_func(void(*func)(unsigned int uart_index, float val, float ms));
void bldc_interface_set_rx_dec_adc_func(void(*func)(unsigned int uart_index, float val, float voltage));
void bldc_interface_set_rx_dec_chuk_func(void(*func)(unsigned int uart_index, float val));

void bldc_interface_set_sim_control_function(void(*func)(unsigned int uart_index, motor_control_mode mode, float value));
void bldc_interface_set_sim_values_func(void(*func)(unsigned int uart_index));

// Setters
void bldc_interface_terminal_cmd(unsigned int uart_index, char* cmd);
void bldc_interface_set_duty_cycle(unsigned int uart_index, float dutyCycle);
void bldc_interface_set_current(unsigned int uart_index, float current);
void bldc_interface_set_current_brake(unsigned int uart_index, float current);
void bldc_interface_set_rpm(unsigned int uart_index, int rpm);
void bldc_interface_set_pos(unsigned int uart_index, float pos);
void bldc_interface_set_handbrake(unsigned int uart_index, float current);
void bldc_interface_set_servo_pos(unsigned int uart_index, float pos);

// Getters
void bldc_interface_get_fw_version(unsigned int uart_index);
void bldc_interface_get_values(unsigned int uart_index);
void bldc_interface_get_mcconf(unsigned int uart_index);
void bldc_interface_get_appconf(unsigned int uart_index);
void bldc_interface_get_decoded_ppm(unsigned int uart_index);
void bldc_interface_get_decoded_adc(unsigned int uart_index);
void bldc_interface_get_decoded_chuk(unsigned int uart_index);

// Other functions
void bldc_interface_detect_motor_param(unsigned int uart_index, float current, float min_rpm, float low_duty);
void bldc_interface_reboot(unsigned int uart_index);
void bldc_interface_send_alive(unsigned int uart_index);
void send_values_to_receiver(unsigned int uart_index, mc_values *values);

// Helpers
const char* bldc_interface_fault_to_string(mc_fault_code fault);

#endif /* BLDC_INTERFACE_H_ */
