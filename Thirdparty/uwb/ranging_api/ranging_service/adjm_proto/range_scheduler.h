#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>

#include "./network_handler.h"
#include "./ipc_proto.h"
#include "../ranging_config.h"
#include "../../common/pretty_printer.h"
#include "../wrapper/cubes_api.h"

/* Scheduler API */
/* please refer to config.h to configurate static parameters*/

// int init_ranging(char * serial_port)
//
//	description:
//		this function initializes all variables to perform 
//		ranging procedure
//	params:
//		serial_port -> the name of the serial port file
//
//	output:
//		The function returns 0 on success, -1 on failure
//
int init_ranging_scheduler(char * serial_port);

// int start_ranging()
//
//	description:
//		this function starts the ranging procedure loop.
//		it's a blocking function so it should be performed
//		on a separate thread or process. 
//	params:
//		[NONE]
//
//	output:
//		The function returns 0 on success, -1 on failure
int start_ranging_scheduler();

// void stop_handler()
//
// this handler should be executed on the main termination.
// Usually it can be inserted inside a signal handler 
void stop_handler();
