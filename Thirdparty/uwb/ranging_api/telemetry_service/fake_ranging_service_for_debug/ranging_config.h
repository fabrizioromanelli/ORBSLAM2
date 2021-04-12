#pragma once

#include "../common/common_config.h"

// NODE LIST
#define NODE_1 0x45bb
#define NODE_2 0x4890
#define NODE_3 0x0C16
#define NODE_4 0x4829

// The Master node is the one who starts the
// adjm proocedure
#define NODE_MASTER NODE_1

// We can't actually work with more than 8 nodes due
// to the UWB firmware limitations.
#define TOTAL_NODES 4
#if TOTAL_NODES > 8
	#error "Max available nodes are 32"
#endif

// Node list as external vector. Defined in range_scheduler.c
extern const uint16_t node_list[];

#define LOOP_DURATION 500 //[msec] time between each adjacency matrix building procedure
#define WAIT_BEFORE_RETRY_ADJM 100 //[msec] if matrix fails, a new attempt is performed after this time
#define ADJM_TIMEOUT 400 //[msec] adjacency matrix timeout


// computes node id index from node id
#define NODE_ID_TO_INDEX(node_id, outindex) \
	for(int i = 0; i < TOTAL_NODES; i++) {\
		if(node_list[i] == (node_id)) {\
			outindex = i;\
		}\
	}

