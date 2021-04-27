// cubes_api.h

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup Types Common data types
 * @{
 */
typedef uint16_t node_id_t;
typedef uint16_t distance_t; ///< Distance in cm
typedef uint32_t timestamp_t;
/**@}*/

/* -- Sending and receiving data ------------------------------------------------------------------------------------ */
/**
 * \defgroup SendRecv Sending and receiving data.
 * @{
 */

#define MAX_PAYLOAD_LEN 55 ///< The maximum app payload length
#define NO_DISTANCE 0xFFFF ///< Adj. matrix value representing a no-distance

/**
 * Outgoing packet queue status
 */
enum out_queue_status {
  OQ_SUCCESS,  ///< the packet has been enqueued, the sent callback will be eventually called
  OQ_FAIL,     ///< unspecified failure (e.g. the module is not initialised), the packet has not been enqueued
  OQ_FULL      ///< the queue is full, the packet has not been enqueued
};

/** Send a packet. Puts a copy of the data payload on the outgoing queue.
 *  If the call is successful, the send status callback will eventually be
 *  called.

 * @param dst Destination address. 0xFFFF and 0x0000 mean broadcast.
 * @param buf Data to send.
 * @param length Length of the data, should be less than #MAX_PAYLOAD_LEN
 * @param handle An arbitrary number to track the packet. It is passed to the sent status callback.
 * @return Outgoing queue status.
 */
enum out_queue_status send(node_id_t dst, const uint8_t* buf, uint8_t length, uint16_t handle);

/**
 * Packet transmission status
 */
enum tx_status {
  TX_SUCCESS, ///< the packet has been transmitted (and acknowledged, if applicable)
  TX_NOACK,   ///< the packet has been transmitted but no acknowledgement arrived (if applicable)
  TX_FAIL     ///< the packet has not been transmitted
};

/** Send status notification callback function pointer type.
 *
 * @param dst The destination node ID
 * @param handle The packet identifier associated in the send() call.
 * @param status The packet transmission status.
 */
typedef void (*sent_cb_t)(node_id_t dst, uint16_t handle, enum tx_status status);

/**
 * Register a callback which is called every time a packet from the outgoing
 * queue is processed. NULL means not interested.
 */
void register_sent_cb(sent_cb_t cb);

/** Receive notification callback function pointer type.
 */
typedef void (*recv_cb_t)();

/**
 * Register a callback which is called every time a packet is received
 * and added to the incoming queue. NULL means not interested.
 */
void register_recv_cb(recv_cb_t cb);

/** Returns the number of packets waiting to be read from the incoming queue
 */
int get_recv_queue_length();

/** Copy the next packet data from the incoming queue to the specified pointer
 *  and remove the packet from the queue. If the queue is empty, do nothing.
 *  The destination memory must be able to accomodate at least #MAX_PAYLOAD_LEN bytes.
 *
 *  @param buf A pointer to memory where the payload will be copied
 *  @param from A pointer to memory where the source node ID will be copied
 *  @return Non-zero length if a payload was copied.
 */
int pop_recv_packet(uint8_t* buf, node_id_t* from);

/**@}*/


/* -- Neighbor table ------------------------------------------------------------------------------------------------ */
/**
 * \defgroup NbrTable Dealing with the neighbor table
 * @{
 */

typedef enum {
  NBR_OK,
  NBR_DELETED
} nbr_status;

/** Neighbor table item
 */
typedef struct {
  node_id_t id;
  distance_t distance;
  timestamp_t last_rng_time;
  timestamp_t last_heard;
  nbr_status status;
} nbr_t;

/** Neighbor info updated callback function pointer type.
 *
 *  Called when
 *  - a neighbor is deleted (e.g. if not heard for a long time)
 *  - a new neighbor is detected and added
 */
typedef void (*nbr_cb_t)(nbr_t nbr, bool deleted);

/** Register a neighbor table callback. NULL means not interested.
 */
void register_nbr_cb(nbr_cb_t nbr_cb);

/** Get the next entry in the neighbor table starting at the given index
 *
 *  @param index The index of the neighbor to start searching from. -1 means
 *  start from the beginning of the table.
 *  @param nbr A pointer in memory where the neighbor info will be copied.
 *  @return Positive index if the operation is successful, -1 otherwise.
 *  The returned value can be used to remove the neighbor or continue iterating
 *  over the table.
 */
int nbr_table_next(int index, nbr_t* nbr);

/** Remove the entry with the specified index from the neighbor table
 *
 *  @param index The index of the neighbor to be removed.
 *  @return Positive index if the operation is successful, -1 otherwise.
 *  The returned value can be used to remove the neighbor or continue iterating
 *  over the table.
 */
int nbr_table_remove(int index);

/** Search the neighbor table for the specific node id
 *
 *  @param nbr A pointer in memory where the neighbor info will be copied.
 *  @return Positive index if the operation is successful, -1 otherwise.
 *  The returned value can be used to remove the neighbor or continue iterating
 *  over the table.
 */
int nbr_get_from_node_id(node_id_t node_id, nbr_t* nbr);

/**@}*/

/* -- Ranging and adjacency matrix ---------------------------------------------------------------------------------- */
/**
 * \defgroup Ranging Ranging and building the adjacency matrix
 * @{
 */

/** Range with a node. The result (if successful) will be communicated via
 *  the ranging callback.
 *
 *  @return Status: FAIL (zero) in case it is impossible to perform the request immediately.
 */
int range_with(node_id_t node);

/** Range with multiple nodes. The result will be communicated via the rall callback.
 *
 *  @param init The initiator of the procedure. If different from the requester,
 *  a request is sent to the specified initiator.
 *  @param nodes The nodes the initiator should range with.
 *  @param num_nodes Number of elements in the array pointed by nodes.
 *  @return Status: FAIL (zero) in case it is impossible to perform the request immediately.
 */
int multi_range_with(node_id_t init, const node_id_t* nodes, uint8_t num_nodes);

/** Ranging done callback. It is called as a response to the ranging request.
 *  @param nbr The neighbor table entry of the node the ranging with performed with.
 *  @param status Non-zero if the ranging was successful.
 */
typedef void (*rng_cb_t)(nbr_t nbr, int status);

/** Register a ranging callback. NULL means not interested.
 *
 *  The callback will be called on request as well as when a ranging was initiated
 *  internally by the ranging subsystem (e.g., when constructing an adjacency matrix)
 */
void register_rng_cb(rng_cb_t cb);

/**
 * Multiple ranging descriptor (RALL requests)
 */
typedef struct {
  int num_nodes;            /**< Number of nodes in the RALL array */

  node_id_t init;           /**< ID of the RALL initiator (the RALL may be a
                            remote request with initiator different from the
                            requesting node) **/

  node_id_t *nodes;         /**< A pointer to the node ID array of size num_nodes
                            listing the nodes participating in the RALL request */
  distance_t *distances;    /**< A pointer to the array of distances associated to
                            the nodes in the RALL request */
  timestamp_t tstamp;       /**< When the RALL was completed (the finish time of
                            the process) */
} rall_descr_t;

/** RALL request (multi_range_with) complete callback function pointer type.
 */
typedef void (*rall_cb_t)(rall_descr_t* rall);

/** Register a RALL callback. NULL means not interested.
 */
void register_rall_cb(rall_cb_t adjm_cb);

/** Adjacency matrix cell data type
 */
typedef struct {
  distance_t distance;
} adjm_cell_t;

/**
 * Adjacency matrix descriptor
 */
typedef struct {
  int num_nodes;       /**< Number of nodes in the adjacency matrix and the nodes
                            array */
  node_id_t *nodes;    /**< A pointer to the node ID array of size num_nodes
                            listing the nodes participating in the matrix, in
                            the order of rows and columns */
  adjm_cell_t *matrix; /**< A pointer to the adjacency matrix, an array of size
                            num_nodes^2, with rows and columns in the order of
                            the nodes array. Cells containing NO_DISTANCE refer
                            to failed ranging sessions. */
  timestamp_t tstamp;  /**< When the matrix was built (the finish time of the
                            matrix construction process) */
} adjm_descr_t;

/** Adjacency matrix update ready callback function pointer type.
 */
typedef void (*adjm_cb_t)(adjm_descr_t* adjm);

/** Register an adjacency matrix callback. NULL means not interested.
 */
void register_adjm_cb(adjm_cb_t adjm_cb);

/**
 * Start the all-to-all ranging procedure among the specified nodes.
 * If NULL, all the nodes from the neighbor table will be used.
 *
 * When the request is done, the callback will be called.
 *
 * Note that this request starts an all-to-all ranging coordinated
 * by the current node. At the end of the ranging session the coordinator
 * will broadcast the adjacency matrix to the rest of the network, so they
 * will update their local copies of the matrix and notify their applications.
 *
 * @param nodes The list of nodes to do the all-to-all ranging with.
 * @param num_nodes The number of nodes in the list.
 * @return Non-zero if the request has been accepted, zero otherwise (e.g., if
 * the system is busy with the ongoing previous matrix request.)
 */
int build_adjm(const node_id_t* nodes, uint8_t num_nodes);

/**@}*/

/* -- Configuration and initialisation  ----------------------------------------------------------------------------- */
/**
 * \defgroup Config System configuration
 * @{
 */

/** Initialise the communication/ranging subsystem.
 *
 *  This function must be called once, before calling any other API
 *  functions.
 *
 *  @param serial_device The system device name of the EVB1000 board
 */
int init(const char *serial_device);

/** Reset the communication/ranging system to a clean state and keep the radio OFF.
 *
 *  @return Non-zero on success. Zero if the UWB hardware module cannot
 *  be put to a pre-defined state.
 */
int power_off();

/** Start the communication/ranging system with the current configuration.
 *
 *  @return Non-zero on success
 */
int power_on();

/** Get the link-layer address (node ID) of the DecaWave device.
 *
 */
node_id_t get_node_id();

#ifdef __cplusplus
}
#endif

/**@}*/
