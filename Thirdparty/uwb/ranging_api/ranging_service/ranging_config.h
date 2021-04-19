#pragma once

#include "wrapper/cubes_api.h"
#include "../common/common_config.h"

// NODE LIST
#define NODE_1 0x45bb
#define NODE_2 0x4890
#define NODE_3 0x0C16
#define NODE_4 0x4829

// The Master node is the one who starts the
// adjm proocedure
#define NODE_MASTER NODE_4

// We can't actually work with more than 8 nodes due
// to the UWB firmware limitations.
#define TOTAL_NODES 4
#if TOTAL_NODES > 8
	#error "Max available nodes are 8"
#endif

// Node list as external vector. Defined in range_scheduler.c
extern const node_id_t node_list[];

// EDIT HERE TO CHANGE SYSTEM TIMING //////////////////////////////////////////////
/*
 * Working configuraiton:
 *	LOOP_DURATION 1000
 *	SLEEP_BEFORE_BROADCAST_BASE 100
 *	SLEEP_BEFORE_BROADCAST_RND 50
 *
 * Experimental configuration:
 *	LOOP_DURATION 500
 *	SLEEP_BEFORE_BROADCAST_BASE 75
 *	SLEEP_BEFORE_BROADCAST_RND 25
 *
 */
#define LOOP_DURATION 500 //[msec] time between each adjacency matrix building procedure
// broadcast configuration ////////////////////////////////////////////////////////
#define SLEEP_BEFORE_BROADCAST_BASE 75 //[msec] minimum time from matrix callback to broadcast
#define SLEEP_BEFORE_BROADCAST_RND 25 //[msec] time to perform ordered send. (multiplied by node index)
#define BROADCAST_FAILURE_RETRY 1 // number of try for broadcast (1 = do not retry)
// adjacency matrix configuration /////////////////////////////////////////////////
#define WAIT_BEFORE_RETRY_ADJM 100 //[msec] if matrix fails, a new attempt is performed after this time
#define ADJM_TIMEOUT 400 //[msec] adjacency matrix timeout
///////////////////////////////////////////////////////////////////////////////////

// enable transmission debbugging. Comment here to disable send failure rate printing
//#define __TX_DBG

// code simplify procedures
#define lock_coordinates \
		if(pthread_mutex_lock(&coordinates_mutex) < 0) {\
			epprint("Error while acquiring coordinates_mutex", get_node_id(), start);\
			exit(-1);\
		}
		
#define unlock_coordinates\
		if(pthread_mutex_unlock(&coordinates_mutex) < 0) {\
			epprint("Error while releasing coordinates_mutex", get_node_id(), start);\
			exit(-1);\
		}

#define lock_active_mask \
		if(pthread_mutex_lock(&activemask_mutex) < 0) {\
			epprint("Error while acquiring activemask_mutex", get_node_id(), start);\
			exit(-1);\
		}
		
#define unlock_active_mask\
		if(pthread_mutex_unlock(&activemask_mutex) < 0) {\
			epprint("Error while releasing activemask_mutex", get_node_id(), start);\
			exit(-1);\
		}

// computes node id index from node id
#define NODE_ID_TO_INDEX(node_id, outindex) \
	for(int i = 0; i < TOTAL_NODES; i++) {\
		if(node_list[i] == (node_id)) {\
			outindex = i;\
		}\
	}

