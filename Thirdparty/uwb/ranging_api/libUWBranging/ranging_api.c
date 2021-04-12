#include "ranging_api.h"

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
ipc_channel_t * start_ipc() {
	return ipc_open(IPC_NAME);
}

int get_data_vectors(ipc_channel_t * c, uint16_t * adjvec, coordinates_t * coord_vector, uint64_t * timestamp) 
{
	// in order to keep atomic read, we have to read the full vector in a single operation

	uint8_t * readbuffer = malloc(c->shm_size);

	int num_of_nodes = get_number_of_nodes(c);
	if(num_of_nodes < 0)
		return -1;

	int res = ipc_read(c, readbuffer, c->shm_size, 0);
	if(res < 0) {
		return -1;
	}
	
	int adjsize = (num_of_nodes * (num_of_nodes - 1) / 2) * sizeof(uint16_t);
	int coordsize = num_of_nodes * sizeof(coordinates_t);
	memcpy(adjvec, 		readbuffer, adjsize);
	memcpy(coord_vector, 	readbuffer + adjsize, coordsize);
	memcpy(timestamp, 	readbuffer + adjsize + coordsize, sizeof(uint64_t));
	
	return 0;
}

int get_number_of_nodes(ipc_channel_t * c) {

	int nodes_num = 0;
	if(ipc_read_constant(c, &nodes_num, sizeof(uint16_t), OFFSET_NUM_OF_NODES) < 0)
		return -1;
	return nodes_num;
}

int get_node_list(ipc_channel_t * c, uint16_t * node_id_list)
{
	int tot_nodes = get_number_of_nodes(c);
	if(tot_nodes < 0)
		return -1;
	return ipc_read_constant(c, node_id_list, tot_nodes * sizeof(uint16_t), OFFSET_NODE_LIST);
}

int write_coordinates(ipc_channel_t * c, double lat, double lon)
{
	coordinates_t coord;
	coord.c_lat = lat;
	coord.c_lon = lon;
	
	int num_of_nodes = get_number_of_nodes(c);
	if(num_of_nodes < 0)
		return -1;
	
	uint16_t nodeindex = 0;
	if(ipc_read_constant(c, &nodeindex, sizeof(uint16_t), OFFSET_OWN_INDEX) < 0)
		return -1;
	
	int offset = (num_of_nodes * (num_of_nodes - 1) / 2) * sizeof(uint16_t) + nodeindex * sizeof(coordinates_t);
	
	return ipc_write(c, &coord, sizeof(coordinates_t), offset);
}

uint16_t get_my_nodeid(ipc_channel_t * c) {

	uint16_t nodeindex = 0;
	if(ipc_read_constant(c, &nodeindex, sizeof(uint16_t), OFFSET_OWN_INDEX) < 0)
		return 0;
	
	uint16_t my_id;
	if(ipc_read_constant(c, &my_id, sizeof(uint16_t), OFFSET_NODE_LIST + nodeindex * sizeof(uint16_t)) < 0)
		return 0;
	
	return my_id;
}
