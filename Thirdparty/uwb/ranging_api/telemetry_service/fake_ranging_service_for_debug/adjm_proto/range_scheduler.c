#include "range_scheduler.h"

ipc_channel_t * channel;

struct timespec start;
uint16_t adjmatrix[TOTAL_NODES * (TOTAL_NODES - 1) / 2];
coordinates_t node_abs_pos[TOTAL_NODES];

const uint16_t node_list[4] = {NODE_1, NODE_2, NODE_3, NODE_4};

int heartbeat = 0;
char heratbeat_char[] = {'_', ' '};

void stop_handler() {
	// This is a signal handler used to close
	// the shared memory file when CTRL+C is
	// pressed.
	close_channel(channel); 
}

int init_ranging_scheduler() {
	// Reset node coordinates memory
	memset(node_abs_pos, 0, TOTAL_NODES * sizeof(coordinates_t));
	memset(adjmatrix, 0, (TOTAL_NODES * (TOTAL_NODES - 1) / 2) * sizeof(uint16_t));
	
	// IPC //
	if(start_channel(&channel, 0xFFFF) < 0) {
		printf("failed to initialize IPC\n");
		return -1;
	}
	return 0;
}

int start_ranging_scheduler() {
	
	struct timespec monitor_before;
	struct timespec monitor_after;

	printf("System ready. Starting protocol\n");
	
	while(1) {
		// register the initial time of the loop to compute
		// the sleeping time.
		clock_gettime(CLOCK_MONOTONIC_RAW, &monitor_before);
		
		/* RECOVER RECEIVED COORDINATES */
		// In order to avoid locking the recv callback, this software won't
		// update the ipc directly through the callback. Instead after each
		// round the received coordinate vector is copied in the channel
		// entirely. The temporary vector is used to avoid deathlocks.
		
		node_abs_pos[0].c_lat++;
		node_abs_pos[0].c_lon--;
		node_abs_pos[1].c_lat++;
		node_abs_pos[1].c_lon--;
		node_abs_pos[2].c_lat++;
		node_abs_pos[2].c_lon--;
		node_abs_pos[3].c_lat++;
		node_abs_pos[3].c_lon--;
		
		if(update_channel(channel, node_abs_pos, adjmatrix, 0, 0xFFFF) < 0) {
			printf("ERROR! cannot write on IPC channel\n");
			return -1;
		}
		
		/* wait for the next round */
		// compute the sleeping time
		clock_gettime(CLOCK_MONOTONIC_RAW, &monitor_after);
		double rng_duration = timespec_to_msec(monitor_after, monitor_before);
		int sleepduration = LOOP_DURATION - (int) rng_duration;
		if(sleepduration < 1)
			sleepduration = 1;
		// wait for at least 1ms if rng_duration >= LOOP_DURATION
		msleep(sleepduration);
		
		// Print the heartbeat
		heartbeat = (heartbeat + 1) % 2;
		printf("%c     duration: %.2f               \n\
		x_0: %.1f\tx_1: %.1f\n\n", heratbeat_char[heartbeat], rng_duration, node_abs_pos[0].c_lat, node_abs_pos[0].c_lon);

	}
	return 0;
}

