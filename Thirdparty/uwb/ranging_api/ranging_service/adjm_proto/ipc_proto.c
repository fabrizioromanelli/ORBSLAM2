#include "ipc_proto.h"

/* All this is needed to avoid recompile kalman filter when updating something
 * ipc_protocol:
 *	byte	|	size	|	description
 *	0	| uint16_t	|	distance NODE_1 NODE_2
 *	2	| uint16_t	|	distance NODE_1 NODE_3
 *	4	| uint16_t	|	distance NODE_1 NODE_4
 *	6	| uint16_t	|	distance NODE_2 NODE_3
 *	8	| uint16_t	|	distance NODE_2 NODE_4
 *	10	| uint16_t	|	distance NODE_3 NODE_4
 *	12	| coordinates_t	|	coordinates of NODE_1
 *	28	| coordinates_t	|	coordinates of NODE_2
 *	44	| coordinates_t	|	coordinates of NODE_3
 *	60	| coordinates_t	|	coordinates of NODE_4
 *	76	| uint64_t	|	timestamp
 *	84	| uint16_t	|	active node bitmask
 *
 *
 * constants:
 *	0	| uint16_t	|	# of nodes
 *	2	| uint16_t	|	index of the current node id in the list
 *	4	| uint16_t	|	NODE_1 id
 *	6	| uint16_t	|	NODE_2 id
 *	8	| uint16_t	|	NODE_3 id
 *	10	| uint16_t	|	NODE_4 id
 *
 */

/////////////////////////INTERNAL APIs///////////////////////////////////////////////
int start_channel(ipc_channel_t ** c, node_id_t my_id)
{	
	if(c == NULL) {
		fprintf(stderr, "[start_channel] Error: invalid input\n");
		return -1;
	}	
	
	uint16_t nodeindex = 0;
	NODE_ID_TO_INDEX(my_id, nodeindex);
	
	uint16_t total_nodes = TOTAL_NODES;
	// generate constant file
	uint8_t init_const[CONST_SIZE];
	memset(init_const, 0, CONST_SIZE);
	memcpy(init_const + NUM_OF_NODES_OFFSET, &total_nodes, sizeof(uint16_t));
	memcpy(init_const + CURRENT_NODE_ID_OFFSET, &nodeindex, sizeof(uint16_t));
	memcpy(init_const + NODE_LIST_OFFSET, node_list, TOTAL_NODES * sizeof(uint16_t));
	
	*c = ipc_init(IPC_NAME, init_const, CONST_SIZE, IPC_SIZE);
	if(*c == NULL) {
		fprintf(stderr, "[start_channel] Error, cannot create the ipc channel\n");
		return -1;
	}
	
	// initial buffer
	uint8_t init_buffer[IPC_SIZE];
	memset(init_buffer, 0, IPC_SIZE);

	// write the timestamp
	uint64_t timestamp = (uint64_t) time(NULL);
	memcpy(init_buffer + TIMESTAMP_OFFSET, &timestamp, sizeof(uint64_t));
	
	if(ipc_write(*c, init_buffer, IPC_SIZE, 0) < 0) {
		fprintf(stderr, "[start_channel] Error, cannot initialize the ipc channel\n");
		return -1;
	}
	
	return 0;
}

void close_channel(ipc_channel_t * c) 
{
	if(c == NULL) {
		fprintf(stderr, "[close_channel] Error: invalid channel\n");
	}	
	
	if(ipc_close(c, IPC_NAME) < 0) {
		fprintf(stderr, "Cannot close the channel	\n");
	}
}

int update_channel(ipc_channel_t * c, coordinates_t * coord_list, uint16_t * distance_list, uint16_t activebitmap, node_id_t my_id)
{
	if(c == NULL) {
		fprintf(stderr, "[update_channel] Error: invalid channel\n");
		return -1;
	}	

	if(coord_list == NULL) {
		fprintf(stderr, "[update_channel] Invalid coordinates list");
		return -1;
	}

	if(distance_list == NULL) {
		fprintf(stderr, "[update_channel] Invalid adjacency matrix");
		return -1;
	}

	uint16_t nodeindex = 0;
	NODE_ID_TO_INDEX(my_id, nodeindex);

	// in order to minimize semaphore acquisition and avoid race condition, 
	// all the data is copied in a buffer
	uint8_t writebuffer[IPC_SIZE];
	
	// distance list 4 * 2 bytes
	memcpy(writebuffer, distance_list, ADJMATRIX_SIZE);
	// coordinates list (4 * 2 * 8 bytes)
	memcpy(writebuffer + COORDS_LIST_OFFSET, coord_list, COORDS_SIZE);
	// timestamp 8 bytes
	uint64_t timestamp = (uint64_t) time(NULL);
	memcpy(writebuffer + TIMESTAMP_OFFSET, &timestamp, sizeof(uint64_t));
	//active mask 1 byte
	memcpy(writebuffer + ACTIVE_MASK_OFFSET, &activebitmap, sizeof(uint16_t));
	
	// we should not write our own position on the IPC
	int size_before = ADJMATRIX_SIZE + nodeindex * sizeof(coordinates_t);
	int offset_after = size_before + sizeof(coordinates_t);
	int size_after = IPC_SIZE - offset_after;
	
	// well, this should not be correct because another thread could write
	// a new set of coordinates. However it will be translated in a delayed
	// writing.
	// If we do not avoid writing own coordinates from this thread we can be 
	// stucked in a 0 loop
	int w1 = ipc_write(c, writebuffer, size_before, 0);
	int w2 = ipc_write(c, writebuffer + offset_after, size_after, offset_after);
	
	if(w1 == 0 && w2 == 0)
		return 0;
	return -1;
}

int get_own_coords(ipc_channel_t * c, coordinates_t * coords, node_id_t my_id)
{
	if(c == NULL) {
		fprintf(stderr, "[get_own_coords] Error: invalid channel\n");
		return -1;
	}	
	uint16_t nodeindex = 0;
	NODE_ID_TO_INDEX(my_id, nodeindex);
	return ipc_read(c, coords, sizeof(coordinates_t), COORDS_LIST_OFFSET + nodeindex * sizeof(coordinates_t));
}

