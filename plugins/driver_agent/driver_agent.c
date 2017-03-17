/*
 Copyright (c) 2012, Broadcom Europe Ltd
 Copyright (c) 2012, OtherCrashOverride
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Video decode demo using OpenMAX IL though the ilcient helper library
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "driver_agent.h"

#define PLUGIN_NAME "driver_agent"

static void release(void *user_data)
{
	free(user_data);
}

static int picam360_driver_xmp(char *buff, int buff_len, float light0_value,
		float light1_value, float motor0_value, float motor1_value,
		float motor2_value, float motor3_value) {
	int xmp_len = 0;

	xmp_len = 0;
	buff[xmp_len++] = 0xFF;
	buff[xmp_len++] = 0xE1;
	buff[xmp_len++] = 0; // size MSB
	buff[xmp_len++] = 0; // size LSB
	xmp_len += sprintf(buff + xmp_len, "http://ns.adobe.com/xap/1.0/");
	buff[xmp_len++] = '\0';
	xmp_len += sprintf(buff + xmp_len, "<?xpacket begin=\"﻿");
	buff[xmp_len++] = 0xEF;
	buff[xmp_len++] = 0xBB;
	buff[xmp_len++] = 0xBF;
	xmp_len += sprintf(buff + xmp_len, "\" id=\"W5M0MpCehiHzreSzNTczkc9d\"?>");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<x:xmpmeta xmlns:x=\"adobe:ns:meta/\" x:xmptk=\"picam360-capture rev1\">");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">");
	xmp_len += sprintf(buff + xmp_len, "<rdf:Description rdf:about=\"\">");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<picam360_driver"
							" light0_value=\"%f\" light1_value=\"%f\""
							" motor0_value=\"%f\" motor1_value=\"%f\" motor2_value=\"%f\" motor3_value=\"%f\""
							" />", light0_value, light1_value, motor0_value,
					motor1_value, motor2_value, motor3_value);
	xmp_len += sprintf(buff + xmp_len, "</rdf:Description>");
	xmp_len += sprintf(buff + xmp_len, "</rdf:RDF>");
	xmp_len += sprintf(buff + xmp_len, "</x:xmpmeta>");
	xmp_len += sprintf(buff + xmp_len, "<?xpacket end=\"w\"?>");
	buff[xmp_len++] = '\0';
	buff[2] = ((xmp_len - 2) >> 8) & 0xFF; // size MSB
	buff[3] = (xmp_len - 2) & 0xFF; // size LSB

	return xmp_len;
}

static void command_handler(void *user_data, char *buff){
	char *cmd;
	cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	} else if (strncmp(cmd, PLUGIN_NAME ".set_light_value", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			char buff[1024];
			int xmp_len = picam360_driver_xmp(buff, sizeof(buff),
					value, value, 0, 0, 0, 0);
			int fd = open("driver", O_RDWR);
			if (fd > 0) {
				write(fd, buff, xmp_len);
				close(fd);
			}
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".set_motor_value", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value;
			sscanf(param, "%f", &value);

			char buff[1024];
			int xmp_len = picam360_driver_xmp(buff, sizeof(buff),
					0, 0, value, value, value, value);
			int fd = open("driver", O_RDWR);
			if (fd > 0) {
				write(fd, buff, xmp_len);
				close(fd);
			}
		}
	} else {
		printf(PLUGIN_NAME ":unknown command : %s\n", buff);
	}
}

void create_driver_agent(PLUGIN_T **_plugin){
	PLUGIN_T *plugin = (PLUGIN_T*)malloc(sizeof(PLUGIN_T));
	strcpy(plugin->name, PLUGIN_NAME);
	plugin->release = release;
	plugin->command_handler = command_handler;
	plugin->user_data = plugin;

	*_plugin = plugin;
}
