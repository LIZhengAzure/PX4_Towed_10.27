/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "XGDVisionPosition.hpp"

#include <lib/drivers/device/Device.hpp>

XGDVisionPosition::XGDVisionPosition(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))

{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/*
	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}
	*/
}

XGDVisionPosition::~XGDVisionPosition()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int XGDVisionPosition::init()
{
	start();

	return PX4_OK;
}

int XGDVisionPosition::collect()
{
	perf_begin(_sample_perf);

	int bytes_processed = 0;

	int index = 0;



	bool checksum_passed = false;

	// Read from the sensor UART buffer.
	//const hrt_abstime timestamp_sample = hrt_absolute_time();
	int bytes_read = ::read(_file_descriptor, &_buffer[0], sizeof(_buffer));
	//printf("cd num:%d===========\r\n",bytes_read);
	if (bytes_read > 0) {
		index = bytes_read - 8;

		while (index >= 0 && !checksum_passed) {
			if (_buffer[index] == VISION_POSITION_PACKET_HDR) {

				bytes_processed = index;

				while (bytes_processed < bytes_read && !checksum_passed) {

					uint8_t checksum_value = _buffer[index] + _buffer[index + 1] + _buffer[index + 2] + _buffer[index + 3] + _buffer[index + 4] +_buffer[index + 5] +_buffer[index + 6];
					uint8_t checksum_byte = _buffer[index + 7];

					if (checksum_value == checksum_byte) {
						checksum_passed = true;
						//printf("sum ture\r\n");
						_distance_x = (_buffer[index + 2] << 8) | _buffer[index + 1];
						_distance_y = (_buffer[index + 4] << 8) | _buffer[index + 3];
						_distance_z = (_buffer[index + 6] << 8) | _buffer[index + 5];
						printf("buff:%d %d %d %d %d %d\r\n",_buffer[1],_buffer[2],_buffer[3],_buffer[4],_buffer[5],_buffer[6]);

						vision_position_s report{};
						report.vision_position_x = _distance_x;
						report.vision_position_y = _distance_y;
						report.vision_position_z = _distance_z;
						_vision_position_topic.publish(report);
					}



					bytes_processed++;
				}
			}

			index--;
		}
	}

	if (!checksum_passed) {
		return -EAGAIN;
	}


	perf_end(_sample_perf);

	return PX4_OK;
}

int XGDVisionPosition::open_serial_port(const speed_t speed)
{
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		PX4_DEBUG("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	if (!isatty(_file_descriptor)) {
		PX4_WARN("not a serial device");
		return PX4_ERROR;
	}

	termios uart_config{};

	// Store the current port configuration. attributes.
	tcgetattr(_file_descriptor, &uart_config);

	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// No parity, one stop bit.
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// No line processing - echo off, echo newline off, canonical mode off, extended input processing off, signal chars off
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void XGDVisionPosition::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	collect();
}

void XGDVisionPosition::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(VISION_POSITION_MEASURE_INTERVAL, 0);
}

void XGDVisionPosition::stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);

	// Clear the work queue schedule.
	ScheduleClear();
}

void XGDVisionPosition::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
