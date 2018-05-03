/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file ahrs.c
 * Hi rate 	Output of attitude data to UART
 *
 * @author Leonid Novitskiy <leonid@wisetech.pro>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
//#include <nuttx/sched.h>
//#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <perf/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <poll.h>


#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

#define TO_GR(a) a / M_PI*180.f

__EXPORT int ahrs_main(int argc, char *argv[]);

int ahrs_uart_config(void);

int ahrs_main(int argc, char *argv[])
{
	PX4_INFO("AHRS start!");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

	int fd_att = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	/* limit the update rate to 50 Hz */
	orb_set_interval(sensor_sub_fd, 20);
	orb_set_interval(fd_att, 20);

	/* advertise attitude topic */
	//struct vehicle_attitude_s att;
	//memset(&att, 0, sizeof(att));
	//orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = fd_att,			.events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 1500; i++) {
		/* wait for sensor update of 2 file descriptor for 100 ms (1 second) */
		int poll_ret = px4_poll(fds, 2, 50);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				//PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
				//	 (double)raw.accelerometer_m_s2[0],
				//	 (double)raw.accelerometer_m_s2[1],
				//	 (double)raw.accelerometer_m_s2[2]);



				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
				/*att.q[0] = raw.accelerometer_m_s2[0];
				att.q[1] = raw.accelerometer_m_s2[1];
				att.q[2] = raw.accelerometer_m_s2[2];

				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);*/
			}
			if (fds[1].revents & POLLIN) {


				struct vehicle_attitude_setpoint_s raw_att;

				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude), fd_att, &raw_att);
				int tstamp 	= raw_att.timestamp;

				PX4_INFO("Attitude data:\ttime:%i\troll:%8.4f\tpitch:%8.4f\tyaw:%8.4f", tstamp,
					 (double)TO_GR(raw_att.roll_body),
					 (double)TO_GR(raw_att.pitch_body),
					 (double)TO_GR(raw_att.yaw_body));
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}


int ahrs_uart_config(void)
{
	char uart_name[] = "/dev/ttyACM0";
	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

		unsigned speed = 921600;

		if (serial_fd < 0) {
			err(1, "failed to open port: %s", uart_name);
		}

		/* Try to set baud rate */
		struct termios uart_config;
		int termios_state;

		/* Back up the original uart configuration to restore it after exit */
		if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
			warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
			close(serial_fd);
			return -1;
		}

		/* Clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* USB serial is indicated by /dev/ttyACM0*/
		if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

			/* Set baud rate */
			if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
				warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
				close(serial_fd);
				return -1;
			}

		}

		if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
			warnx("ERR SET CONF %s\n", uart_name);
			close(serial_fd);
			return -1;
		}
	return serial_fd;

}
