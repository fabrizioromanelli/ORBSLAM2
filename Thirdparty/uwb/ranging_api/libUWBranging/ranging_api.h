#pragma once

#include "../common/ipc_interface.h"
#include "../common/common_config.h"

#define OFFSET_NUM_OF_NODES 0
#define OFFSET_OWN_INDEX sizeof(uint16_t)
#define OFFSET_NODE_LIST 2*sizeof(uint16_t)


/** returns a struct that represents the ipc channel associated with
 * the telemetry data.
 
 * @return on success the address of a ipc_channel_t struct is returned, on failure NULL il returned
 */
ipc_channel_t * start_ipc();

/** returns the last computed adjacency matrix for this node.
 * This operation is guaranteed to be atomic with respect of the data production

 * @param ipc_channel_t structure that represent an open ipc channel
 * @param adjvec adjacency vector. Each entry represent the distance from one node 
 *		to the others. The order is: 12, 13, ..., 1N, 23, 24, ..., 2N ...
 *		(i.e. for 4 nodes:
 *			12, 13, 14, 23, 24, 34
 *		 that represents:
 *			- 12 13 14
 *			-  - 23 24
 *			-  -  - 34
 *			-  -  -  -		
 *		)
 *		the size of the vector is computed with:  
 *		get_number_of_nodes() * (get_number_of_nodes() - 1) / 2.
 * @param coord_vector coordinate vector. Each entry represent the distance from one node. The
 *		is defined by get_node_list()
 * @param timestamp timestamp of the last computation of the vector
 * @return 0 on success, -1 on error
 */
int get_data_vectors(ipc_channel_t * c, uint16_t * adjvec, coordinates_t * coord_vector, uint64_t * timestamp);

/** returns the ordered list of nodes. The list size can be obtained
 * by get_number_of_nodes()

 * @param ipc_channel_t structure that represent an open ipc channel 
 * @param node_id_list pointer to the node id list
 * @return 0 on success, -1 on error
 */
int get_node_list(ipc_channel_t * c, uint16_t * node_id_list);

/** returns the number of nodes defined in config.h

 * @param c ipc_channel_t structure that represent an open ipc channel 
 * @return number of nodes
 */
int get_number_of_nodes(ipc_channel_t * c);

/** This software handles only coordinates communication between nodes
 * however the coordinates should be provided by the GPS management
 * module and/or the kalman filter

 * @param ipc_channel_t structure that represent an open ipc channel 
 * @param lat node latitude
 * @param lon node longitude
 * @return 0 on success, -1 on error
 */
int write_coordinates(ipc_channel_t * c, double lat, double lon);

/** This function returns the id of the calling node.
 
 * @param ipc_channel_t structure that represent an open ipc channel
 * @return id of the connected node
 */
uint16_t get_my_nodeid(ipc_channel_t * c);

