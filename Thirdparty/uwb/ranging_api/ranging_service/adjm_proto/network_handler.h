#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>

#include "../wrapper/cubes_api.h"
#include "../ranging_config.h"

#define SYNC_SUCCESS 0
#define RANGE_COLLISION 3
#define SEND_FAILED 4
#define LOCK_ERROR 5

#define ERROR_NOT_CALLABLE -1
#define ERROR_TIMEOUT -2
#define ERROR_SYSTEM -3
#define ERROR_NO_RESPONSE -4
#define ERROR_SEND_FAILED -5

#define BROADCAST_ADDRESS 0xFFFF

#define RANGING_TIMEOUT 100
#define SEND_TIMEOUT 100

void init_callbacks();
int sync_send(node_id_t, uint8_t *, int);
int sync_broadcast(uint8_t *, int);
