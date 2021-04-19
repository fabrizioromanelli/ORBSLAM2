#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

#include "../libUWBranging/ranging_api.h"

/***************************
 * RANGING BENCHMARK:
 * This software is used to test the reception capabilities of the UWB
 * modules by computing the percentage of missing packets. It sends an
 * incremental value over the medium and checks if it reads a incremental
 * value from the other nodes (all the nodes must run this executable)
 *
 * In this way three files are created inside the shmfs: 
 * telemetry_const, telemetry_shm, sem.telemetrysem
 *
 * These files represents a POSIX IPC channel and the relative semaphore. However
 * the API provided here should garant that you doesn't need to read directly
 * the files.
 *
 * IPC files are created only by the ranging_api process.
 * Moreover these files are created in exclusive mode, so
 * no ranging process can begin if the files exists.
 *
 * Please check and delete them if they already exists before starting the
 * ranging executable.
 */
 
int loop_exit = 0;

void sigint_handler(int signum) {
	(void) signum;
	loop_exit = 1;
}

#define DEFAULT_LOOP_TIME 1000000

int main(int argc, char **argv) {

	int loop_time = DEFAULT_LOOP_TIME;
	
	if(argc == 2) {
		char * p;
		int msec_pause;
		errno = 0;
		
		// Loop pause can be obtained from starting parameters
		msec_pause = strtol(argv[1], &p, 10);
		if(errno != 0 || *p != '\0') {
			fprintf(stderr, "Error: invalid input parameters\n");
			exit(-1);
		}
		loop_time = msec_pause * 1000;
	}
	
	// IPC INIT ///////////////////////////////////////////
	/* 
	 * If the start_ipc() fails, probably the ranging process
	 * is not in execution. Wait for it for a bit and then fail
	 */
	int ipc_test_counter = 0;
	ipc_channel_t * channel = NULL;
	do {
		channel = start_ipc();
		if(channel == NULL) {
			fprintf(stderr, "IPC not ready.\n");
			if(ipc_test_counter < 5){
				fprintf(stderr, "retry\n");
			}
			else {
				fprintf(stderr, "maximum attempt reached\n");
				exit(1);
			}
			ipc_test_counter++;
			usleep(1000000);
		}
	} while(channel == NULL);
	
	
	signal(SIGINT, sigint_handler);
	/* Here the channel has been opened. We can begin reading
	 * constants and configure all we need to print data.
	 */
	
	// number of nodes
	int number_of_nodes = get_number_of_nodes(channel);
	if(number_of_nodes < 0) {
		fprintf(stderr, "Error, invalid number of nodes\n");
		exit(1);
	}
	
	// nodes list
	uint16_t * nodes_list = malloc(number_of_nodes * sizeof(uint16_t));
	if(nodes_list == NULL) {
		fprintf(stderr, "memory allocation error\n");
		exit(1);
	}
	if(get_node_list(channel, nodes_list) < 0) {
		fprintf(stderr, "Error, invalid node list\n");
		exit(1);
	}
	
	// my id
	uint16_t my_id = get_my_nodeid(channel);
	if(my_id == 0) {
		fprintf(stderr, "Error, invalid ID\n");
		exit(1);
	}
	
	uint16_t my_index = 0;
	for(int i = 0; i < number_of_nodes; i++) {
		if(my_id == nodes_list[i]) {
			my_index = i;
		}
	}

	/* vector allocation: here adjacency vector and coordinates
	 * vector are preallocated to be filled in the main loop
	 */
	uint16_t * adjacency_vector = malloc(number_of_nodes * (number_of_nodes - 1) /  2 * sizeof(uint16_t));
	if(adjacency_vector == NULL) {
		fprintf(stderr, "OutOfMemoryError: cannot allocate adjacency vector\n");
		exit(1);
	}
	coordinates_t * coords_vector = malloc(number_of_nodes * sizeof(coordinates_t));
	if(adjacency_vector == NULL) {
		fprintf(stderr, "OutOfMemoryError: cannot allocate adjacency vector\n");
		exit(1);
	}
	uint64_t timestamp;
	
	double lat_demo = 0.0;
	double lon_demo = 0.0;
	
	
	/* We have read all the constant information we need.
	 * here we can start our protocol
	 */
	fprintf(stdout, "Initialization completed!\n");
	
    	if(get_data_vectors(channel, adjacency_vector, coords_vector, &timestamp) < 0) {
			fprintf(stderr, "ERROR! cannot read shared memory\n");
			exit(1);
		}
	
	int prev_reading[number_of_nodes];
	int tot_reading[number_of_nodes];
	int tot_errors[number_of_nodes];
	int max_burst = 0;
	for(int i = 0; i < number_of_nodes; i++) {
		prev_reading[i] = coords_vector[i].c_lat;
		tot_reading[i] = 0;
		tot_errors[i] = 0;
	}
	
	// MAIN LOOP ////////////////////////////////////////////////////////////////////
	while(loop_exit == 0) {
		/* each round get_data_vector() must be called. it
		 * doesn't matter how much time passes between two
		 * queries because the data is always available and
		 * always protected by the semaphore.
		 * However the timestamp value should be used as a 
		 * reference to check if new data is available.
		 */
		if(get_data_vectors(channel, adjacency_vector, coords_vector, &timestamp) < 0) {
			fprintf(stderr, "ERROR! cannot read shared memory\n");
			exit(1);
		}
		else {
			/* to evaluate the reception error we send incremental data. Then
			 * if we observe a step greather than a single increment while
			 * reading data, a reception error has been observed.
			 */
			for(int i = 0; i < number_of_nodes; i++) {
				if(i == my_index) {
					continue;
				}
				
				if(coords_vector[i].c_lat > prev_reading[i] + 1) {
					tot_errors[i] += coords_vector[i].c_lat - (prev_reading[i] + 1);
					if(max_burst < coords_vector[i].c_lat - (prev_reading[i] + 1))
						max_burst = coords_vector[i].c_lat - (prev_reading[i] + 1);
				}
				
				tot_reading[i] += coords_vector[i].c_lat - prev_reading[i];
				prev_reading[i] = coords_vector[i].c_lat;
			}
		}
		
		if(write_coordinates(channel, lat_demo, lon_demo) < 0) {
			fprintf(stderr, "ERROR! cannot write on shared memory\n");
			exit(1);
		}
		
		fprintf(stdout, "Error perc:\t");
		for(int i = 0; i < number_of_nodes; i++) {
			if(tot_reading[i] == 0)
				fprintf(stdout, " 0");
			else
				fprintf(stdout, "\t%.2f", ((double)tot_errors[i]) / tot_reading[i] * 100);
		}
		fprintf(stdout, "\n");
		
		fprintf(stdout, "lat vector:\t");
		for(int i = 0; i < number_of_nodes; i++) {
			fprintf(stdout, "\t%.2f", (coords_vector[i].c_lat));
		}
		fprintf(stdout, "\n\n");

		lat_demo = lat_demo + 1;
		lon_demo = lon_demo + 1;
		
		usleep(loop_time);
	}
	
	// STATISTICS AND LOGGING ///////////////////////////////////////////////////
	float max_err = 0.0, min_err = tot_reading[my_index];
	int min_transmissions = tot_reading[my_index];
	for(int i = 0; i < number_of_nodes; i++) {
		if(i != my_index) {
			float ith_error = ((double)tot_errors[i]) / tot_reading[i] * 100;
			if(ith_error > max_err)
				max_err = ith_error;
			if(ith_error < min_err)
				min_err = ith_error;
			if(tot_reading[i] < min_transmissions)
				min_transmissions = tot_reading[i];
		}
			
	}
	
	
	FILE * logfile = fopen("./range_benchmark_log.txt", "a");
	if(logfile == NULL) {
		fprintf(stderr, "ERROR: cannot open logfile\n");
		exit(-1);
	}
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	
	fprintf(logfile, "Log date: %d/%d/%d %d:%d\n", tm.tm_mday, tm.tm_mon, tm.tm_year + 1900, tm.tm_hour, tm.tm_min);
	fprintf(logfile, "Max error: %.2f%%\nMin error: %.2f%%\nMinimum number of transmissions: %d\n\n", max_err, min_err, min_transmissions);
	
	fclose(logfile);
	
	exit(0);
	
}
