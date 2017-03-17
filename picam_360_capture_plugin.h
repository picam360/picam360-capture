#pragma once

typedef struct _PLUGIN_T{
	char name[64];
	void (*release)(void *user_data);
	void (*command_handler)(void *user_data, char *cmd);
	void *user_data;
} PLUGIN_T;

typedef void (*CREATE_PLUGIN)(PLUGIN_T **plugin);

