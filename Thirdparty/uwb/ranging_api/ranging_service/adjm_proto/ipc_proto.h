#pragma once

#include <string.h>

#include "../ranging_config.h"
#include "../wrapper/cubes_api.h"
#include "../../common/ipc_interface.h"

#define CONST_SIZE ((TOTAL_NODES + 2) * sizeof(int))
#define ADJMATRIX_SIZE ((TOTAL_NODES * (TOTAL_NODES - 1) / 2) * sizeof(uint16_t))
#define COORDS_SIZE (TOTAL_NODES * sizeof(coordinates_t))

#define IPC_SIZE ADJMATRIX_SIZE + COORDS_SIZE + sizeof(uint64_t) + sizeof(uint16_t)

#define ADJM_LIST_OFFSET 0
#define COORDS_LIST_OFFSET (ADJM_LIST_OFFSET + ADJMATRIX_SIZE)
#define TIMESTAMP_OFFSET (COORDS_LIST_OFFSET + COORDS_SIZE)
#define ACTIVE_MASK_OFFSET (TIMESTAMP_OFFSET + sizeof(uint64_t))

#define NUM_OF_NODES_OFFSET 0
#define CURRENT_NODE_ID_OFFSET (NUM_OF_NODES_OFFSET + sizeof(uint16_t))
#define NODE_LIST_OFFSET (CURRENT_NODE_ID_OFFSET + sizeof(uint16_t))

void close_channel(ipc_channel_t * c);
int start_channel(ipc_channel_t ** c, node_id_t my_id);
int update_channel(ipc_channel_t * c, coordinates_t * coord_list, uint16_t * distance_list, uint16_t activebitmap, node_id_t my_id);
int get_own_coords(ipc_channel_t * c, coordinates_t * coords, node_id_t my_id);

