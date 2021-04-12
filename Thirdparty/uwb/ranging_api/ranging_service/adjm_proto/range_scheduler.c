#include "range_scheduler.h"

ipc_channel_t * channel;

struct timespec start;
int activemask = 0;
uint16_t * adjmatrix = NULL;
coordinates_t * node_abs_pos = NULL;
coordinates_t owncoord; // store coordinates that should be passed to the thread

pthread_mutex_t coordinates_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t activemask_mutex = PTHREAD_MUTEX_INITIALIZER;

// adjacency matrix wait conditions
pthread_mutex_t adjm_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t adjm_cond = PTHREAD_COND_INITIALIZER;

node_id_t my_id;
int myidx;
int masteridx;

int discovered_nodes = 1;

const node_id_t node_list[] = {NODE_1, NODE_2, NODE_3, NODE_4};

int heartbeat = 0;
char heratbeat_char[] = {'_', ' '};

#ifdef __TX_DBG
int failed_transmission = 0;
int total_transmission = 0;
#endif

void stop_handler() {
	// This is a signal handler used to close
	// the shared memory file when CTRL+C is
	// pressed.
	close_channel(channel);
	
	free(node_abs_pos);
	free(adjmatrix);
}

void broadcast_position() {
	/* This funciton is used to perform coordinates broadcast.
	 * The broadcast is pseudo-ordered to avoid collision.
	 * Each node in the network will wait for an amount of time
	 * proportional to its index in the node list
	 */

	// This is necessary to avoid that the last node is the one who
	// perform the last broadcast. In this case a casual delay could
	// break the next adjacency matrix generation.
	int ordered_wait = ((myidx - masteridx + 2) % 4) * SLEEP_BEFORE_BROADCAST_RND;
	
	msleep(SLEEP_BEFORE_BROADCAST_BASE + ordered_wait);

	// Try to broadcast own position. 
	int broadcast_status = -1000;
	int retry = BROADCAST_FAILURE_RETRY;
	do {
		broadcast_status = send(BROADCAST_ADDRESS, (uint8_t *)  &(owncoord), sizeof(coordinates_t), 15);
		if(broadcast_status != OQ_SUCCESS) {
			dpprint("Broadcast failed", my_id, start);
		    msleep(SLEEP_BEFORE_BROADCAST_RND / 2);
        }
		retry--;
	}
	while(retry != 0 && broadcast_status != OQ_SUCCESS);
	
#ifdef __TX_DBG
	if(broadcast_status != OQ_SUCCESS) {
		failed_transmission++;
	}
	total_transmission++;
#endif
}

void snd_scheduler(node_id_t dst, uint16_t handle, enum tx_status status) 
{
	(void) dst;
	(void) handle;
	(void) status;
	return;
}

static void recv_scheduler() {
	
	node_id_t src;
	uint8_t buf[MAX_PAYLOAD_LEN];
	int recv_len = pop_recv_packet(buf, &src);

	if(recv_len == sizeof(coordinates_t)) {
		int nodeidx = 0;
		NODE_ID_TO_INDEX(src, nodeidx);
		
		// coordinates lock here are used to update absolute positions
		// of other nodes. This prevents error while exposing them
		// on a external API
		lock_coordinates
		
		memcpy(&(node_abs_pos[nodeidx]), buf, sizeof(coordinates_t));
		
		unlock_coordinates
	}
}

static void nbr_scheduler(nbr_t node, bool deleted) {
	
	int nodeidx = 0;
	NODE_ID_TO_INDEX(node.id, nodeidx);
	
	if(deleted) {
		pprint("neighbour node %X deleted", my_id, start, node.id);
		lock_active_mask
		discovered_nodes--;
		UNSET_BIT(activemask, nodeidx);
		unlock_active_mask
	}
	else {
		pprint("neighbour node %X discovered", my_id, start, node.id);
		lock_active_mask
		discovered_nodes++;
		SET_BIT(activemask, nodeidx);
		unlock_active_mask
	}
}

static void adjm_scheduler(adjm_descr_t* adjm)
{
	if(pthread_mutex_lock(&adjm_mutex) < 0) {
		dpprint("Cannot get adjmatrix lock", my_id, start);
	}
	
	/* TRANSLATION ZONE
	 * due to the request of the upper level, the following code has the purpose
	 * to provide the space for the entire adjacency matrix.
	 * However the symmetric parts and the diagonals (all 0s) are ignored.
	 * for example with 4 nodes the order is:
	 * 12, 13, 14, 
	 *     23, 24, 
	 *         34
	 * and the size of this vector is n * (n - 1) / 2
	 */
	 
	// Map the nodes receved into our order.
	// i.e. node_order_map[3] = 0 means that
	// NODE_4 is the first node of the received matrix
	int node_order_map[TOTAL_NODES] = {-1, -1, -1, -1};
	for(int i = 0; i < TOTAL_NODES; i++) {
		for(int j = 0; j < adjm->num_nodes; j++) {
			if(adjm->nodes[j] == node_list[i]) {
				node_order_map[i] = j;
			}
		}
	}
	
	// Then write the correct adjacency upper-triangular matrix
	int adjsize = TOTAL_NODES * (TOTAL_NODES - 1) / 2;
	int i1 = 0;
	int i2 = 1;
	for(int i = 0; i < adjsize; i++) {
		
		int matrix_node_x = node_order_map[i2];
		int matrix_node_y = node_order_map[i1];
		
		
		if(matrix_node_x == -1 || matrix_node_y == -1)
		 	// inactive node. The UNAVAILABLE_NODE macro is a 
		 	// wrapper for UINT16_MAX that is also used for 
		 	// failed matrix building
			adjmatrix[i] = UNAVAILABLE_NODE;
		else {
			// The adjacency matrix should not be symmetric, so we have two measures
			// for each couple of nodes. We can compute the mean value to reduce
			// the total error.
			
			// x * n + y
			int matrix_location = matrix_node_x * adjm->num_nodes + matrix_node_y;
			// y * n + x
			//int specular_location = matrix_node_y * adjm->num_nodes + matrix_node_x;
			// compute distance mean
			// TODO: check here the correctness of the measurement
			int d_up = (adjm->matrix)[matrix_location].distance;
			adjmatrix[i] = d_up;
			//int d_dw = (adjm->matrix)[specular_location].distance;
			//adjmatrix[i] = (d_up + d_dw) / 2;
		}
		i2 = (i2 + 1) % TOTAL_NODES;
		if(i2 ==  0) {
			i1++;
			i2 = i1 + 1;
		}
	}
	
	if(pthread_cond_signal(&adjm_cond) < 0) {
		dpprint("Cannot signal the adjmatrix cond variable", my_id, start);
	}
	
	if(pthread_mutex_unlock(&adjm_mutex) < 0) {
		dpprint("Cannot release adjmatrix lock", my_id, start);
	}
	
	broadcast_position();
}

int init_ranging_scheduler(char * serial_port) {
	clock_gettime(CLOCK_MONOTONIC_RAW, &start);

	// This is not fundamental, however in order to perform a clean start
	// we can turn off the radio before performing the init
	// TODO: check if the power_on() function is needed
	power_off();

	if (!init(serial_port)) {
		printf("Failed to initialize\n");
		return -1;
	}
	
	// allocate memory on the heap (for thread sharing)
	adjmatrix = malloc(TOTAL_NODES * (TOTAL_NODES - 1) / 2 * sizeof(uint16_t));
	if(adjmatrix == NULL) {
		printf("Failed to allocate matrix memory");
		return -1;
	}
	// set each point to the maximum value (the adjmatrix is unsigned so
	// we fill it with 1)
	memset(adjmatrix, 0xFF, TOTAL_NODES * (TOTAL_NODES - 1) / 2 * sizeof(uint16_t));
	
	node_abs_pos = calloc(TOTAL_NODES, sizeof(coordinates_t));
	if(node_abs_pos == NULL) {
		printf("Failed to allocate coordinates memory");
		return -1;
	}
	
	// IPC //
	my_id = get_node_id();
	if(start_channel(&channel, my_id) < 0) {
		printf("failed to initialize IPC\n");
		return -1;
	}

	// Save own index to avoid recomputing it on each iteration
	NODE_ID_TO_INDEX(my_id, myidx);
	// Keep the master index too. It's useful for ordinate send
	// and it's constant.
	NODE_ID_TO_INDEX(NODE_MASTER, masteridx);
	
	// INTRO //
	printintro(my_id, serial_port);

	//init_callbacks();
	register_sent_cb(snd_scheduler);
	register_nbr_cb(nbr_scheduler);
	register_recv_cb(recv_scheduler);
	register_adjm_cb(adjm_scheduler);
	
	return 0;
}

int start_ranging_scheduler() {

	if(node_abs_pos == NULL || adjmatrix == NULL) {
		dpprint("ERROR! uninitialized system.", my_id, start);
		return -1;
	}
	
	struct timespec monitor_before;
	struct timespec monitor_after;
	
	// if there is only 1 node, the adjm will fail. Until the first neighbor
	// callback is called, the adjmatrix function won't return success.
	int atomic_mask;
	int atomic_discovered;
	do {
		lock_active_mask
		atomic_discovered = discovered_nodes;
		atomic_mask = activemask;
		unlock_active_mask
		msleep(500);
		dpprint("No other nodes here. Waiting...", my_id, start);
	}
	while(atomic_discovered == 1);
	
	pprint("System ready. Starting protocol", my_id, start);
	
	while(1) {
		// register the initial time of the loop to compute
		// the sleeping time.
		clock_gettime(CLOCK_MONOTONIC_RAW, &monitor_before);
		
		/* RETRIVE OWN COORDINATES */
		// the coordinates should have been written by the kalman filter process
		// on the IPC channel. The get_own_coords() function will query the IPC
		// channel to obtain this data.
		// The only thread who can write this node absolute position
		// is the main thread so the lock is not necessary here to access
		// the data. Here we update the position with the one obtained
		// by the kalman filter through the IPC channel

		if(get_own_coords(channel, &(node_abs_pos[myidx]), my_id) < 0) {
			dpprint("ERROR! cannot read IPC channel", my_id, start);
			return -1;
		}
		
		/* BUILD ADJACENCY MATRIX */
		// get lock
		if(pthread_mutex_lock(&adjm_mutex) < 0) {
			dpprint("Cannot get adjmatrix lock", my_id, start);
		}
		
		// owncoord is synchronized inside the condition zone
		memcpy(&owncoord, &(node_abs_pos[myidx]), sizeof(coordinates_t));
		
		// if we are the master node, start the procedure
		if(my_id == NODE_MASTER) {
			if(!build_adjm(NULL, 4)) {
				dpprint("ERROR! adjacency matrix call failed", my_id, start);
				if(pthread_mutex_unlock(&adjm_mutex) < 0) {
					dpprint("Cannot release adjmatrix lock", my_id, start);
				}
				// If the adjacency matrix fails, a shorter time is waited before
				// retrying. Each other procedure is skipped.
				msleep(WAIT_BEFORE_RETRY_ADJM);
				continue;
			}
			struct timespec timeout;
			clock_gettime(CLOCK_REALTIME, &timeout);
			timespec_add_msec(timeout, ADJM_TIMEOUT);
			// in either cases wait for a callback
			if(pthread_cond_timedwait(&adjm_cond, &adjm_mutex, &timeout) < 0) {
				dpprint("Pthread cond wait failed on adjm condition variable", my_id, start);
			}
		}
		else {
			if(pthread_cond_wait(&adjm_cond, &adjm_mutex) < 0) {
				dpprint("Pthread cond wait failed on adjm condition variable", my_id, start);
			}
		}
		// release lock
		if(pthread_mutex_unlock(&adjm_mutex) < 0) {
			dpprint("Cannot release adjmatrix lock", my_id, start);
		}
		
		
		/* RECOVER RECEIVED COORDINATES */
		// In order to avoid locking the recv callback, this software won't
		// update the ipc directly through the callback. Instead after each
		// round the received coordinate vector is copied in the channel
		// entirely. The temporary vector is used to avoid deathlocks.
		coordinates_t coords_tmp[TOTAL_NODES];
		
		lock_coordinates
		memcpy(coords_tmp, node_abs_pos, TOTAL_NODES * sizeof(coordinates_t));
		unlock_coordinates
		
		
		if(update_channel(channel, coords_tmp, adjmatrix, atomic_mask, my_id) < 0) {
			dpprint("ERROR! cannot write on IPC channel", my_id, start);
			return -1;
		}
		
		
		// Only the master node should sleep the LOOP_DURATION due to the fact that
		// the other nodes are stuck in the pthread cond wait. If the other nodes are
		// sleeping too, the callback could interrupt the sleeping and then they wait for the
		// next condition ignoring the first one.
		if(my_id == NODE_MASTER) {
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
			pprint("%c     duration: %.2f               ", my_id, start, heratbeat_char[heartbeat], rng_duration);
		}
		else {
			// the other nodes don't need to monito loop duration
			heartbeat = (heartbeat + 1) % 2;
			pprint("%c                                  ", my_id, start, heratbeat_char[heartbeat]);
		}
		#ifdef __TX_DBG
			if(total_transmission != 0){
				fprintf(stderr, "\nTransmission failure rate: %.2f \n", ((double) failed_transmission) / total_transmission * 100.0);
			}
		#endif
	}
	return 0;
}

