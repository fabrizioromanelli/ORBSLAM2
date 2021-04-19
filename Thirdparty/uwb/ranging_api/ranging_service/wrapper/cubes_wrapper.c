#include "cubes_api.h"
#include "cubes_common.h"
#include "serial.h"
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

#if DEBUG
#define DEBUG_SERIAL 0
#endif

//#define ADDR_SIZE 2 // length in bytes of the device address

/* Serial communication */
#define SERIAL_BUF_LEN 2048 // size of the buffer for wrapper-module communication
#define MAX_MODULE_PACKET_LEN 128 // max length of a message (for both data and control packets)

/* Neighbors management */
#define NBR_AUTOREMOVE_PERIOD 10 // number of seconds between checks for automatic deletion of old neighbors
#define NBR_TIMEOUT_DEFAULT 20 // time before deleting a node from the neighbors table

#define IS(cmd) (strncmp((char*)input, #cmd, 4) == 0)
#define IS_OK()   (strncmp((char*)input+5, "_OK_", 4) == 0)
#define IS_NOA()  (strncmp((char*)input+5, "NO_A", 4) == 0)
#define IS_OFF()  (strncmp((char*)input+5, "OFF_", 4) == 0)

static enum {
  ST_IDLE,
  ST_SENDING,
  ST_RANGING,
  ST_RANGING_ALL,
  ST_ADJM,
  ST_ADJM_RECV,
  ST_ADJM_BC
} state;

/* system initialization flag */
bool is_initialized = false;

/* Serial communication between module and wrapper */
int serial_fd;
pthread_t serial_tid;

// Request data
uint16_t send_handle;
node_id_t send_dst;
node_id_t ranging_with;

// XXX temporary stuff before we have a queue implemented
node_id_t recv_src_id;
uint16_t recv_len;
uint8_t recv_buf[MAX_MODULE_PACKET_LEN];
uint8_t recv_qlen;
pthread_mutex_t recv_mutex = PTHREAD_MUTEX_INITIALIZER;

// callbacks
recv_cb_t recv_callback;
sent_cb_t sent_callback;
rng_cb_t  rng_callback;
adjm_cb_t adjm_callback;
rall_cb_t rall_callback;
nbr_cb_t nbr_callback;

// device information
node_id_t device_id;

// neighbors table
nbr_t neighbors[MAX_NODES];
pthread_mutex_t nbr_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_t autoremove_tid;

// multi-ranging and adjacency matrix
adjm_descr_t adjm_complete;
adjm_descr_t adjm;
rall_descr_t rall_complete;
rall_descr_t rall;
//node_id_t adjm_current_rall_init;
uint8_t current_rall_init_index;

/*---------------------------------------------------------------------------*/

timestamp_t get_timestamp(void);
int decode_dall(const char *input, uint16_t in_len);
int decode_recv(const char* input, uint16_t in_len);
int decode_adjm_init(const char *input, uint16_t in_len);
bool decode_adjm(const char *input, uint16_t in_len);
void nbr_table_init(void);
int nbr_table_set(nbr_t nbr);
int nbr_table_num(void);
int nbr_table_all_node_ids(node_id_t** nbr_nodes);
int pop_recv_packet(uint8_t* buf, node_id_t* from);
void process_packet(const char *input, uint16_t len);
int index_from_node_id(node_id_t node_id, const node_id_t* node_ids, uint8_t num_nodes);
void prepare_rall_request(node_id_t init, node_id_t* nodes, uint8_t num_nodes);
void prepare_rall_serial(int encoded_length, char* buffer);
int rall_update_adjm(rall_descr_t *rall);
int rall_update_nbr_table(rall_descr_t *rall);
void* serial_reader(void* p);
void* nbr_table_autoremove(void* p);
static int send_serial(char* packet, int len);

/*---------------------------------------------------------------------------*/

timestamp_t get_timestamp(void) {
  struct timespec tstamp;

  clock_gettime(CLOCK_MONOTONIC_RAW, &tstamp);
  return (tstamp.tv_sec * 1000) + (tstamp.tv_nsec / 1000000); // combine sec and nanosec, return millisec

}

int decode_recv(const char* input, uint16_t in_len) {
  node_id_t src_id;
  uint16_t len;

  if (in_len < 14 || input[4] != ' ' || input[9] != ' ')
    return 0;
  if (!decode_hex(input+5, 4, (char*)&src_id))
    return 0;
  if (!decode_hex(input+10, 4, (char*)&len))
    return 0;

  len = ntohs(len);
  src_id = ntohs(src_id);

  if (len == 0) {
    recv_len = len;
    recv_src_id = src_id;
    return 1;
  }
  if (len > MAX_MODULE_PACKET_LEN)
    return 0;
  if ((in_len-15)/2 != len)
    return 0;

  char tmp[len];
  if (decode_hex(input+15, in_len-15, tmp)) {
    recv_len = len;
    recv_src_id = src_id;
    memcpy(recv_buf, tmp, len);
    return 1;
  }

  return 0;
}

int decode_dall(const char *input, uint16_t in_len) {
  int in_read = 5; // ignore 'RALL' and the following space
  uint16_t rall_init;
  uint16_t result_node;
  double result_distance;
  int distance_bytes;
  int result_num = 0;

  PRINTF("Decoding DALL (1): in_len %d, input %s\n", in_len, input);

  /* retrieve initiator */
  if (decode_hex(input+in_read, 4, (char*)&rall_init)) {
    rall.init = ntohs(rall_init);
    in_read += 5;

    PRINTF("DALL init is %04X\n", rall.init);
  }
  else {
    return 0;
  }

  /* as long as there is something to decode */
  while (in_len - in_read >= 4) {
    PRINTF("Decode state: in_read %d, result_num %d, num_nodes %d\n", in_read, result_num, rall.num_nodes);
    if (decode_hex(input+in_read, 4, (char*)&result_node)) {
      PRINTF("Decoded node %04X\n", result_node);
      in_read += 5;
      if (sscanf(input+in_read, "%lf%n", &result_distance, &distance_bytes)) {
        PRINTF("Decoded distance %lf (%d bytes)\n", result_distance, distance_bytes);
        in_read += distance_bytes + 1; // %n in the scan saves the number of bytes read so far, +1 for space
        rall.nodes[result_num] = ntohs(result_node);
        if (result_distance < 0) {
          rall.distances[result_num] = 0;
        }
        else {
          rall.distances[result_num] = result_distance * 100; // meters to cm
        }
        PRINTF("Decoded values assigned\n");
        result_num++;
      }
      else break;
    }
    else break;
  }

  rall.num_nodes = result_num;

  return result_num;
}

int decode_adjm_init(const char *input, uint16_t in_len) {
  int in_read = 5; // ignore 'ADJM'
  node_id_t rall_init;
  node_id_t result_node;
  distance_t result_distance;
  int num_bytes;
  int result_num = 0;
  int num_nodes;
  int i, j;

  /* initialize matrix to save received results */
  if (!sscanf(input+in_read, "%d%n", &num_nodes, &num_bytes)) { // will be ignored in initialization
    PRINTF("Dec. ADJM init: cannot decode current id in %s\n", input);

    return 0;
  }
  in_read += num_bytes + 1;
  if (!sscanf(input+in_read, "%d%n", &num_nodes, &num_bytes)) {
    PRINTF("Dec. ADJM init: cannot decode num_nodes in %s\n", input);

    return 0;
  }
  in_read += num_bytes + 1;
  if (!decode_hex(input+in_read, 4, (char*)&rall_init)) {
    PRINTF("Dec. ADJM init: cannot decode RALL init in %s\n", input);

    return 0;
  }
  in_read += 5;
  //adjm_current_rall_init = rall_init;
  adjm.num_nodes = num_nodes;
  if (adjm.nodes != NULL) free(adjm.nodes);
  adjm.nodes = (node_id_t*)malloc(adjm.num_nodes * sizeof(*(adjm.nodes)));
  if (adjm.matrix != NULL) free(adjm.matrix);
  adjm.matrix = (adjm_cell_t*)malloc(adjm.num_nodes * adjm.num_nodes * sizeof(*(adjm.matrix)));

  /* initialize matrix with max of uint16_t (meaning no result was obtained) */
  for (i = 0; i < adjm.num_nodes; i++) {
    for (j = 0; j < adjm.num_nodes; j++) {
      adjm.matrix[i * adjm.num_nodes + j].distance = NO_DISTANCE;
    }
  }

  PRINTF("Decoding ADJM init (1): in_len %d, input %s, num_nodes %d\n", in_len, input, adjm.num_nodes);

  /* as long as there is something to decode */
  while ((in_len - in_read >= 4) && result_num < adjm.num_nodes) {
    PRINTF("Decode state: in_read %d, result_num %d, num_nodes %d\n", in_read, result_num, adjm.num_nodes);
    if (decode_hex(input+in_read, 4, (char*)&result_node)) {
      result_node = ntohs(result_node);
      PRINTF("Decoded node %04X\n", result_node);
      in_read += 5;
      if (sscanf(input+in_read, "%hu%n", &result_distance, &num_bytes)) {
        PRINTF("Decoded distance %hu (%d bytes)\n", result_distance, num_bytes);
        in_read += num_bytes + 1; // %n in the scan saves the number of bytes read so far, +1 for space
        adjm.nodes[result_num] = result_node;
        result_num++;
      }
      else break;
    }
    else break;
  }

  return result_num;
}

bool decode_adjm(const char *input, uint16_t in_len) {
  int in_read = 5; // ignore 'ADJM'
  node_id_t rall_init;
  node_id_t result_node;
  distance_t result_distance;
  int num_bytes;
  int current_num_node;
  int num_nodes;
  int node_index_init;
  int node_index_resp;

  /* initialize matrix to save received results */
  if (!sscanf(input+in_read, "%d%n", &current_num_node, &num_bytes)) {
    PRINTF("Dec. ADJM: cannot decode current id in %s\n", input);

    return 0;
  }
  in_read += num_bytes + 1;
  if (!sscanf(input+in_read, "%d%n", &num_nodes, &num_bytes)) {
    PRINTF("Dec. ADJM: cannot decode num_nodes in %s\n", input);

    return 0;
  }
  in_read += num_bytes + 1;
  if (!decode_hex(input+in_read, 4, (char*)&rall_init)) {
    PRINTF("Dec. ADJM: cannot decode RALL init in %s\n", input);

    if (current_num_node == 0 && num_nodes == 0) {
      PRINTF("Received timeout indication from module\n");

      return 1;
    }

    return 0;
  }
  rall_init = ntohs(rall_init);
  in_read += 5;

  /* retrieve index of the RALL initiator for this ADJM message */
  node_index_init = index_from_node_id(rall_init, adjm.nodes, adjm.num_nodes);
  if (node_index_init == -1) {
    PRINTF("Dec. ADJM: cannot find index of node %04X\n", rall_init); // should never happen

    return false;
  }

  PRINTF("Dec. ADJM (1): in_len %d, input %s, num_nodes %d\n", in_len, input, adjm.num_nodes);

  /* as long as there is something to decode */
  while ((in_len - in_read >= 4)) {
    if (decode_hex(input+in_read, 4, (char*)&result_node)) {
      result_node = ntohs(result_node);
      PRINTF("Decoded node %04X\n", result_node);
      in_read += 5;
      if (sscanf(input+in_read, "%hu%n", &result_distance, &num_bytes)) {
        PRINTF("Decoded distance %hu (%d bytes)\n", result_distance, num_bytes);
        in_read += num_bytes + 1; // %n in the scan saves the number of bytes read so far, +1 for space

        /* retrieve index for the node in the message */
        node_index_resp = index_from_node_id(result_node, adjm.nodes, adjm.num_nodes);
        if (node_index_resp == -1) {
          PRINTF("Dec. ADJM: cannot find index of node %04X\n", rall_init); // should never happen

          return false;
        }

        /* assign result distance in the matrix */
        adjm.matrix[node_index_init * adjm.num_nodes + node_index_resp].distance = result_distance;

      }
      else break;
    }
    else break;
  }

  return current_num_node == num_nodes; // if they are the same, the ADJM message is the last one
}

void nbr_table_init(void) {
  int i;

  pthread_mutex_lock(&nbr_mutex);
  for (i = 0; i < MAX_NODES; i++) {
    neighbors[i].status = NBR_DELETED;
  }
  pthread_mutex_unlock(&nbr_mutex);
}

int nbr_table_set(nbr_t nbr) {
  int i;
  int deleted_index = -1;

  PRINTF("NEIGHBOR TO SET: %04X\n", nbr.id);

  pthread_mutex_lock(&nbr_mutex);
  for (i = 0; i < MAX_NODES; i++) {
    if (neighbors[i].id == nbr.id && neighbors[i].status != NBR_DELETED) {
      neighbors[i] = nbr;
      pthread_mutex_unlock(&nbr_mutex);

      PRINTF("NEIGHBOR WAS FOUND\n");

      return i;
    }
    if (neighbors[i].status == NBR_DELETED && deleted_index == -1) {
      deleted_index = i;
    }
  }
  if (deleted_index != -1) {
    neighbors[deleted_index] = nbr;
    pthread_mutex_unlock(&nbr_mutex);

    PRINTF("NEIGHBOR SET AT INDEX: %d (%04X)\n", deleted_index, neighbors[deleted_index].id);

    return deleted_index;
  }
  pthread_mutex_unlock(&nbr_mutex);
  return -1;
}

int nbr_table_next(int index, nbr_t* nbr) {
  PRINTF("Requested next node from index %d\n", index);

  if (index >= MAX_NODES - 1) return -1;
  if (index < -1) index = -1;
  index++;

  pthread_mutex_lock(&nbr_mutex);
  for (; index < MAX_NODES; index++) {
    if (index >= MAX_NODES - 1) {
      pthread_mutex_unlock(&nbr_mutex);

      PRINTF("Not found\n");

      return -1;
    }
    if (neighbors[index].status != NBR_DELETED) {
      nbr->id = neighbors[index].id;
      nbr->distance = neighbors[index].distance;
      nbr->last_rng_time = neighbors[index].last_rng_time;
      nbr->last_heard = neighbors[index].last_heard;
      nbr->status = neighbors[index].status;
      pthread_mutex_unlock(&nbr_mutex);

      PRINTF("Found node at index %d: %04X\n", index, nbr->id);

      return index;
    }
  }
  pthread_mutex_unlock(&nbr_mutex);
  return -1;
}

int nbr_table_remove(int index) {
  nbr_t nbr_removed;

  if (index >= MAX_NODES || index < 0) return -1;

  pthread_mutex_lock(&nbr_mutex);
  if (neighbors[index].status != NBR_DELETED) {
    nbr_removed = neighbors[index];
    neighbors[index].status = NBR_DELETED;
    pthread_mutex_unlock(&nbr_mutex);

    /* callback for deleted neighbor */
    if (nbr_callback) {
      nbr_callback(nbr_removed, 1); // 1 means the neighbor was deleted
    }

    return index;
  }
  pthread_mutex_unlock(&nbr_mutex);
  return -1;
}

int nbr_get_from_node_id(node_id_t node_id, nbr_t* nbr) {
  int i;

  pthread_mutex_lock(&nbr_mutex);
  for (i = 0; i < MAX_NODES; i++) {
    if (neighbors[i].id == node_id && neighbors[i].status != NBR_DELETED) {
      nbr->id = neighbors[i].id;
      nbr->distance = neighbors[i].distance;
      nbr->last_rng_time = neighbors[i].last_rng_time;
      nbr->last_heard = neighbors[i].last_heard;
      nbr->status = neighbors[i].status;

      pthread_mutex_unlock(&nbr_mutex);
      return i;
    }
  }
  pthread_mutex_unlock(&nbr_mutex);
  return -1;
}

int nbr_table_all_node_ids(node_id_t** nbr_nodes) {
  int i;
  int nbr_i = 0;
  int num_nodes = 0; // the function will also attach the id of this device

  if (*nbr_nodes != NULL) {
    return -1;
  }

  pthread_mutex_lock(&nbr_mutex);

  /* count current neighbors */
  for (i = 0; i < MAX_NODES; i++) {
    if (neighbors[i].status != NBR_DELETED
    && neighbors[i].id != device_id) {
      num_nodes++;

      PRINTF("Found neighbor in table: i %d, node %04X, total %d\n", i, neighbors[i].id, num_nodes);
    }
  }

  /* allocate an array for ids of neighbors and this device */
  *nbr_nodes = (node_id_t *)malloc((num_nodes + 1) * sizeof(node_id_t));

  /* copy this device id */
  (*nbr_nodes)[nbr_i] = device_id;
  nbr_i++;

  PRINTF("Device id added: i 0, idx %d, node %04X\n", nbr_i - 1, *nbr_nodes[nbr_i - 1]);

  /* copy neighbor ids */
  for (i = 0; i < MAX_NODES; i++) {
    if (neighbors[i].status != NBR_DELETED
    && neighbors[i].id != device_id) {
      (*nbr_nodes)[nbr_i] = neighbors[i].id;
      nbr_i++;

      PRINTF("Neighbor id added: i %d, idx %d, node %04X\n", i, nbr_i - 1, (*nbr_nodes)[nbr_i - 1]);
    }
  }
  pthread_mutex_unlock(&nbr_mutex);

  PRINTF("Neighbors total: %d\n", (num_nodes + 1));

  return (num_nodes + 1);
}

void* nbr_table_autoremove(void* p) {
  timestamp_t current_time;
  nbr_t nbr_removed;
  int i;

  PRINTF("Autoremove thread spawned\n");

  do {
    sleep(NBR_AUTOREMOVE_PERIOD);

    PRINTF("Autoremove in progress\n");

    pthread_mutex_lock(&nbr_mutex);
    current_time = get_timestamp();
    for (i = 0; i < MAX_NODES; i++) {
      if (neighbors[i].status != NBR_DELETED) {
        if (current_time - neighbors[i].last_heard > NBR_TIMEOUT_DEFAULT * 1000) {
          nbr_removed = neighbors[i];
          neighbors[i].status = NBR_DELETED;
          pthread_mutex_unlock(&nbr_mutex);

          /* callback for deleted neighbor */
          if (nbr_callback) {
            nbr_callback(nbr_removed, 1); // 1 means the neighbor was deleted
          }
        }
      }
    }
    pthread_mutex_unlock(&nbr_mutex);
    PRINTF("Autoremove completed\n");
  } while(1);
}

int rall_update_nbr_table(rall_descr_t *rall_info) {
  int updated_num = 0;
  int i;
  nbr_t nbr_recv;

  for (i = 0; i < rall_info->num_nodes; i++) {
    nbr_recv.id = rall_info->nodes[i];
    nbr_recv.distance = rall_info->distances[i];
    nbr_recv.last_rng_time = rall_info->tstamp;
    nbr_recv.last_heard = rall_info->tstamp;
    nbr_recv.status = NBR_OK;
    if (nbr_recv.id != device_id) {
      if (nbr_table_set(nbr_recv) != -1) {
        updated_num++;
      }
    }
  }

  return updated_num;
}

int rall_update_adjm(rall_descr_t *rall) {
  int updated_num = 0;
  int i;
  int rall_init_index;
  int node_index;
  int matrix_location;

  rall_init_index = index_from_node_id(rall->init, adjm.nodes, adjm.num_nodes);
  if (rall_init_index == -1) {
    PRINTF("Err, cannot update adjm (init %04X not found)\n", rall->init);
#if DEBUG
    for (i = 0; i < adjm.num_nodes; i++) {
      PRINTF("\tadjm.nodes[%d] = %04X\n", i, adjm.nodes[i]);
    }
#endif

    return 0;
  }

  PRINTF("Init index is %d\n", rall_init_index);

  for (i = 0; i < rall->num_nodes; i++) {
    node_index = index_from_node_id(rall->nodes[i], adjm.nodes, adjm.num_nodes);
    if (node_index != -1) {
      matrix_location = rall_init_index * adjm.num_nodes + node_index;
      if (node_index != rall_init_index) {
        adjm.matrix[matrix_location].distance = rall->distances[i];
        updated_num++;
      }
    }
    PRINTF("Updating (%d): idx %d, loc %d, dist %d\n", i, node_index, matrix_location, adjm.matrix[matrix_location].distance);
  }

  for (i = 0; i < rall->num_nodes; i++) {
    matrix_location = rall_init_index * adjm.num_nodes + rall_init_index;
    adjm.matrix[matrix_location].distance = 0;
  }

  PRINTF("Updated total: %d\n", updated_num);

  return updated_num;
}

void process_packet(const char *input, uint16_t len) {
  if (IS(CERR)) { // XXX: should never happen
    power_off();

    /* flush serial data (both directions) */
    usleep(10000); // required for USB serial flush
    tcflush(serial_fd,TCIOFLUSH);
    usleep(10000); // required for USB serial flush

    power_on();
    state = ST_IDLE;
  }
  else if (IS(RECV)) {
    int deliver_to_app;
    int nbr_index;
    nbr_t nbr_recv;
    nbr_t nbr_known;
    bool is_nbr_known;

    /* decode input and establish if it should be delivered to the application */
    pthread_mutex_lock(&recv_mutex);
    deliver_to_app = decode_recv(input, len);
    deliver_to_app = deliver_to_app && (recv_len <= MAX_PAYLOAD_LEN) && (recv_len > 0);
    recv_qlen = 1;
    pthread_mutex_unlock(&recv_mutex);

    /* update the neighbors table */
    nbr_recv.id = recv_src_id;
    nbr_recv.last_heard = get_timestamp();
    if (nbr_get_from_node_id(nbr_recv.id, &nbr_known) == -1) { // the node is not present
      nbr_recv.status = NBR_OK;
      nbr_recv.last_rng_time = 0;
      is_nbr_known = false;

      PRINTF("New neighbor.\n");
    }
    else { // the node was already known
      nbr_recv.status = nbr_known.status;
      nbr_recv.distance = nbr_known.distance;
      nbr_recv.last_rng_time = nbr_known.last_rng_time;
      is_nbr_known = true;
    }
    nbr_index = nbr_table_set(nbr_recv);

    /* callback for new neighbor */
    if (nbr_index >=0 && !is_nbr_known && nbr_callback) {
      nbr_callback(nbr_recv, 0); // 0 means a new node was added
    }

    /* callback for application data */
    if (deliver_to_app && recv_callback) {
      recv_callback();
    }
  }
  else if (IS(SEND)) {
    if (!IS_OK()) {
      state = ST_IDLE;
    }
  }
  else if (IS(SENT)) {
    if (state == ST_SENDING) {
      state = ST_IDLE; // XXX use mutex?
      if (sent_callback) {
        if (IS_OK())
          sent_callback(send_dst, send_handle, TX_SUCCESS);
        else if (IS_NOA())
          sent_callback(send_dst, send_handle, TX_NOACK);
        else
          sent_callback(send_dst, send_handle, TX_FAIL);
      }
    }
  }
  else if (IS(DIST)) {
    if (state == ST_RANGING) {
      state = ST_IDLE;

      if (rng_callback) { // TODO: not only if there is a callback...
        nbr_t nbr;
        node_id_t node;

        nbr.id = 0;
        nbr.distance = 0;

        if (decode_hex(input+10, 4, (char*)&node)) {
          nbr.id = ntohs(node);

          if (IS_OK()) {
            double distance;

            if (sscanf(input+15, "%lf", &distance)) {
              nbr_t nbr_recv;

              /* update the neighbors table */
              nbr_recv.id = node;
              if (distance < 0) {
                nbr_recv.distance = 0;
              }
              else {
                nbr_recv.distance = distance * 100; // meters to cm
              }

              nbr_recv.last_rng_time = get_timestamp();
              nbr_recv.last_heard = nbr_recv.last_rng_time;
              nbr_recv.status = NBR_OK;
              nbr_table_set(nbr_recv);

              /* save the node as a neighbor, or update it if present */
              nbr_table_set(nbr_recv);

              /* callback for successful ranging */
              rng_callback(nbr_recv, 1);
            }
            else {/*decode error: should not happen*/
              rng_callback(nbr, 0);
            }
          }
          else {
            rng_callback(nbr, 0);
          }
        }
        else {/*decode error: should not happen*/}
      }
    }
  }
  else if (IS(DALL)) {
    PRINTF("Received DALL (state is %d)\n", state);

    if (state == ST_RANGING_ALL || state == ST_ADJM) {
      uint8_t rall_num_results = decode_dall(input, len);
      if (rall_num_results >= 0) {

        /* both in case of RALL or matrix construction, update neighbors */
        if (rall_num_results >= 1) {
          rall.tstamp = get_timestamp();
          rall_update_nbr_table(&rall);
        }

        if (state == ST_RANGING_ALL) {

          /* deep copy of the RALL structure */
          rall_complete.num_nodes = rall.num_nodes;
          rall_complete.init = rall.init;
          if(rall_complete.nodes != NULL) free(rall_complete.nodes);
          rall_complete.nodes = (node_id_t*)malloc(rall.num_nodes * sizeof(*(rall.nodes)));
          memcpy(rall_complete.nodes, rall.nodes, rall.num_nodes * sizeof(*(rall.nodes)));
          if(rall_complete.distances != NULL) free(rall_complete.distances);
          rall_complete.distances = (distance_t*)malloc(rall.num_nodes * sizeof(*(rall.distances)));
          memcpy(rall_complete.distances, rall.distances, rall.num_nodes * sizeof(*(rall.distances)));
          rall_complete.tstamp = get_timestamp();

          state = ST_IDLE;

          /* callback for completed RALL procedure */
          if (rall_callback) {
            rall_callback(&rall_complete);
          }
        }
        else if (state == ST_ADJM) {

          /* update the adjacency matrix with RALL results */
          if (rall_num_results >= 1) {
            rall_update_adjm(&rall);
          }

          PRINTF("---> Finished RALL %d (tot: %d)\n", current_rall_init_index, adjm.num_nodes);

          current_rall_init_index++;

          /* if all RALL requests were completed */
          if (current_rall_init_index >= adjm.num_nodes) {
            int j;
            char buf[SERIAL_BUF_LEN+1];
            int buf_i = 0;
            int length;

            state = ST_ADJM_BC;

            /* deep copy of the ADJM structure */
            adjm_complete.num_nodes = adjm.num_nodes;
            if(adjm_complete.nodes != NULL) free(adjm_complete.nodes);
            adjm_complete.nodes = (node_id_t*)malloc(adjm.num_nodes * sizeof(*(adjm.nodes)));
            memcpy(adjm_complete.nodes, adjm.nodes, adjm.num_nodes * sizeof(*(adjm.nodes)));
            if(adjm_complete.matrix != NULL) free(adjm_complete.matrix);
            adjm_complete.matrix = (adjm_cell_t*)malloc(adjm.num_nodes * adjm.num_nodes * sizeof(*(adjm.matrix)));
            memcpy(adjm_complete.matrix, adjm.matrix, adjm.num_nodes * adjm.num_nodes * sizeof(*(adjm.matrix)));
            adjm_complete.tstamp = get_timestamp();

            /* broadcast completed matrix */
            PRINTF("Broadcasting matrix\n");

            usleep(10000); // XXX: ensures the receivers is ready after ranging procedures

            current_rall_init_index = 0;

            /* ADJM <index> <total nodes> <RALL init> */
            length = sprintf(
              &buf[buf_i],
              "AMBC %d %d %04X",
              current_rall_init_index + 1,
              adjm_complete.num_nodes,
              adjm_complete.nodes[current_rall_init_index]
            );
            buf_i += length;

            /* attach nodes and distances relatively to the node at index i */
            for (j = 0; j < adjm_complete.num_nodes; j++) {
              length = sprintf(
                &buf[buf_i],
                " %04X %d",
                adjm_complete.nodes[j],
                adjm_complete.matrix[current_rall_init_index * adjm_complete.num_nodes + j].distance
              );
              if (length < 0) {
                PRINTF("Failed to write to buffer (idx %d)\n", buf_i);

                break;
              }
              buf_i += length;

              PRINTF("Current: %s\n", buf);
            }

            /* add new line */
            buf[buf_i] = '\n';
            buf_i++;

            /* transmit ADJM <index> <total nodes> <RALL init> <node-distance list> */
            send_serial(buf, buf_i);
          }
          /* if there are other RALL procedures to perform */
          else {
            PRINTF("Send another RALL\n");

            int encoded_length = 4 + (5 * adjm.num_nodes) + 1; // 4 for request, 5 for init, 5 for each node id, +1 is for \n
            char buffer[encoded_length + 1];  // +1 is for \0

            prepare_rall_request(adjm.nodes[current_rall_init_index], adjm.nodes, adjm.num_nodes);
            prepare_rall_serial(encoded_length, buffer);

            /* transmit RALL <initiator> <other nodes> */
            send_serial(buffer, encoded_length);
          }
        }
      }
    }
  }
  else if (IS(AMBC)) {
    if (state == ST_ADJM_BC) {
      if (IS_OK()) {
        int j;
        char buf[SERIAL_BUF_LEN+1];
        int buf_i = 0;
        int length;

        /* transmit the vector for the following node */
        current_rall_init_index++;

        if (current_rall_init_index < adjm_complete.num_nodes) {

          /* ADJM <index> <total nodes> <RALL init> */
          length = sprintf(
            &buf[buf_i],
            "AMBC %d %d %04X",
            current_rall_init_index + 1,
            adjm_complete.num_nodes,
            adjm_complete.nodes[current_rall_init_index]
          );
          buf_i += length;

          /* attach nodes and distances relatively to the node at index i */
          for (j = 0; j < adjm_complete.num_nodes; j++) {
            length = sprintf(
              &buf[buf_i],
              " %04X %d",
              adjm_complete.nodes[j],
              adjm_complete.matrix[current_rall_init_index * adjm_complete.num_nodes + j].distance
            );
            if (length < 0) {
              PRINTF("Failed to write to buffer (idx %d)\n", buf_i);

              break;
            }
            buf_i += length;

            PRINTF("Current: %s\n", buf);
          }

          /* add new line */
          buf[buf_i] = '\n';
          buf_i++;

          /* transmit ADJM <index> <total nodes> <RALL init> <node-distance list> */
          send_serial(buf, buf_i);
        }
        else if (current_rall_init_index == adjm_complete.num_nodes) {
          state = ST_IDLE;

          /* callback for completed adjacency matrix */
          if(adjm_callback) {
            adjm_callback(&adjm_complete);
          }
        }
      }
      else {
        state = ST_IDLE;
      }
    }
  }
  else if (IS(ADJM)) {
    PRINTF("Received a matrix vector (wrapper in state %d)\n", state);

    /* if it is the first piece of the matrix we received */
    if (state == ST_IDLE) {
      PRINTF("Initialization of ADJM structure\n");

      /* initialize the ADJM structure */
      if (decode_adjm_init(input, len) == 0) {
        return;
      }
      state = ST_ADJM_RECV;
    }

    /* if ready to receive the matrix packets, decode and store */
    if (state == ST_ADJM_RECV) {
      bool is_complete;

      is_complete = decode_adjm(input, len);

      if (is_complete) {
        PRINTF("Complete matrix received!\n");

        /* deep copy of the ADJM structure */
        adjm_complete.num_nodes = adjm.num_nodes;
        if(adjm_complete.nodes != NULL) free(adjm_complete.nodes);
        adjm_complete.nodes = (node_id_t*)malloc(adjm.num_nodes * sizeof(*(adjm.nodes)));
        memcpy(adjm_complete.nodes, adjm.nodes, adjm.num_nodes * sizeof(*(adjm.nodes)));
        if(adjm_complete.matrix != NULL) free(adjm_complete.matrix);
        adjm_complete.matrix = (adjm_cell_t*)malloc(adjm.num_nodes * adjm.num_nodes * sizeof(*(adjm.matrix)));
        memcpy(adjm_complete.matrix, adjm.matrix, adjm.num_nodes * adjm.num_nodes * sizeof(*(adjm.matrix)));
        adjm_complete.tstamp = get_timestamp();

        state = ST_IDLE;

        if (adjm_callback) {
          adjm_callback(&adjm_complete);
        }
      }
    }
  }
  else if (IS(RNGE)) {
    if (!IS_OK()) {
      state = ST_IDLE;
    }
  }
#if DEBUG && DEBUG_SERIAL
  PRINTF("Wrapper state %d\n", state);
#endif
}

void* serial_reader(void* p) {
  char buf[SERIAL_BUF_LEN+1];
  int idx;
  int truncated;
  do {
    char c;
    int rdlen;
    idx = 0;
    truncated = 0;
    do {
      rdlen = read(serial_fd, &c, 1);
      if (rdlen > 0) {
        if (idx < SERIAL_BUF_LEN)
          buf[idx++] = c;
        else
          truncated = 1;
        if (c == '\n')
          break; // complete packet
      } else if (rdlen == 0) {
        printf("Zero bytes read\n");
      }
      else {
        printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        sleep(1); // XXX might have lost the serial connection. Try to reopen? Inform the app?
      }
    } while (1);
    buf[idx] = '\0';
    if (truncated) {
      printf("Truncated serial packet: %s\n", buf); // drop truncated packets
    }
    else {
#if DEBUG && DEBUG_SERIAL
      PRINTF("From serial: %s", buf);
#endif
      process_packet(buf, idx-1 /* to cut the '\n' */);
    }
  } while (1);
}

static int send_serial(char* packet, int len) {
  int wlen = write(serial_fd, packet, len);
  if (wlen != len) {
      printf("Error from write: %d, %d\n", wlen, errno);
      return 0;
  }
  tcdrain(serial_fd);    /* delay for output */

#if DEBUG
  PRINTF("To serial: (%d)", wlen);
  int i;
  for (i = 0; i < wlen; i++) {
    PRINTF(" (%d)%c", i, packet[i]);
  }
  PRINTF("\n");
#endif

  return 1;
}

int init(const char *serial_device) {

  if (is_initialized) return 0;

  /* clear ranging structures */
  rall.num_nodes = 0;
  rall.nodes = NULL;
  rall.distances = NULL;
  rall_complete.num_nodes = 0;
  rall_complete.nodes = NULL;
  rall_complete.distances = NULL;
  adjm.num_nodes = 0;
  adjm.nodes = NULL;
  adjm.matrix = NULL;
  adjm_complete.num_nodes = 0;
  adjm_complete.nodes = NULL;
  adjm_complete.matrix = NULL;

  /* enter initial state */
  state = ST_IDLE;

  /* initialize neighbors table */
  nbr_table_init();

  /* set app callbacks to NULL */
  recv_callback = NULL;
  sent_callback = NULL;
  rng_callback = NULL;
  adjm_callback = NULL;
  rall_callback = NULL;
  nbr_callback = NULL;

  /* open serial port */
  serial_fd = serial_open(serial_device);
  if (serial_fd < 0) {
    printf("Error opening %s: %s\n", serial_device, strerror(errno));
    return 0;
  }
  printf("Port is open: %s\n", serial_device);

  /* serial configuration */
  if (!serial_set_interface_attribs(serial_fd, B115200)) {
    printf("Error setting port attributes\n");
    return 0;
  }

  /* reset module and device */
  power_off();

  /* flush serial data (both directions) */
  usleep(10000); // required for USB serial flush
  tcflush(serial_fd,TCIOFLUSH);
  usleep(10000); // required for USB serial flush

  /* prepare serial configuration for the address request */
  struct termios request_attr;
  struct termios default_attr;
  tcgetattr(serial_fd, &default_attr);
  request_attr = default_attr;
  request_attr.c_cc[VMIN] = 10; // will stop reading when 10 bytes are received
  request_attr.c_cc[VTIME] = 0; // or after a while (tenths of sec) without transmissions
  if (tcsetattr(serial_fd, TCSANOW, &request_attr) != 0) {
    printf("Error from tcsetattr: %s\n", strerror(errno));
    return 0;
  }

  /* request the device address */
  char request[6];
  char input[11];
  int rdlen;
  snprintf(request, 5, "ADDR");
  request[4] = '\n';
  request[5] = '\0';
  device_id = 0;
  if (!send_serial(request, 5)) {
    return 0;
  }
  rdlen = read(serial_fd, input, sizeof(input));
  if (rdlen == 0) {
    printf("Error from read: %d bytes\n", rdlen);
    sleep(1);
    return 0;
  }
  if (rdlen < 0) {
    printf("Error from read: %d: %s\n", rdlen, strerror(errno));
    sleep(1);
    return 0;
  }

  /* save the address locally */
  if (IS(ADDR)) {
    node_id_t addr;
    if (decode_hex(input+5, 4, (char*)&addr)) {
      device_id = ntohs(addr);

      PRINTF("Acquired device id: %04X\n", device_id);
    }
    else {
      printf("Error in decoding module's reply.\n");
      return 0;
    }
  }
  else {
    printf("Error in serial reading: %s\n", (char*)input);
    return 0;
  }

  /* restore default serial configuration */
  if (tcsetattr(serial_fd, TCSANOW, &default_attr) != 0) {
    printf("Error from tcsetattr: %s\n", strerror(errno));
    return 0;
  }

  /* create the dedicated thread for the serial input */
  if (pthread_create(&serial_tid, NULL, serial_reader, NULL)) {
    printf("Error starting a thread (1)\n");
    return 0;
  }

  /* create the dedicated thread for the serial input */
  if (pthread_create(&autoremove_tid, NULL, nbr_table_autoremove, NULL)) {
    printf("Error starting a thread (2)\n");
    return 0;
  }

  /* start the module */
  power_on();

  is_initialized = true;

  state = ST_IDLE;
  return 1;
}

enum out_queue_status send(node_id_t dst, const uint8_t* buf, uint8_t length, uint16_t handle) {
  if (state != ST_IDLE)
    return OQ_FULL;
  if (length > MAX_PAYLOAD_LEN)
    return OQ_FAIL;

  int encoded_length = 15 + length*2 + 1; // +1 is for \n
  char packet[encoded_length + 1];        // +1 is for \0

  // TODO: acquire mutex
  snprintf(packet, 16, "SEND %04X %04X ", dst, length);
  encode_hex((const char*)buf, length, packet+15);
  packet[encoded_length-1] = '\n';
  packet[encoded_length] = '\0';
  //printf("%s", packet);
  if (!send_serial(packet, encoded_length)) {
    return OQ_FAIL;
  }
  state = ST_SENDING;
  send_handle = handle;
  send_dst = dst;
  // TODO: release mutex

  return OQ_SUCCESS;
}

int range_with(node_id_t node) {
  if (node == 0 || node == 0xFFFF)  // don't range with a bcast address
    return 0;
  if (state != ST_IDLE) {
    PRINTF("Err, state %d\n", state);
    return 0;
  }

  int encoded_length = 9 + 1;       // +1 is for \n
  char packet[encoded_length + 1];  // +1 is for \0

  snprintf(packet, encoded_length+1, "RNGE %04X\n", node);
  if (!send_serial(packet, encoded_length)) {
    return 0;
  }
  state = ST_RANGING;

  return 1;
}

void prepare_rall_request(node_id_t init, node_id_t* nodes, uint8_t num_nodes) {
  int i;

  PRINTF("Preparing RALL: init %04X, num_nodes %u\n", init, num_nodes);

  /* clear RALL data */
  if (rall.nodes != NULL) {
    free(rall.nodes);
    rall.nodes = NULL;
  }
  if (rall.distances != NULL) {
    free(rall.distances);
    rall.distances = NULL;
  }

  /* load the information for the procedure in the RALL structure */
  rall.init = init;
  rall.num_nodes = num_nodes;
  rall.nodes = (node_id_t *)malloc(rall.num_nodes * sizeof(node_id_t));
  rall.distances = (distance_t *)malloc(rall.num_nodes * sizeof(distance_t));

  /* store nodes in the RALL structure */
  for (i = 0; i < rall.num_nodes; i++) {
    rall.nodes[i] = nodes[i];

    PRINTF("Adding node %d to rall struct: %04X\n", i, rall.nodes[i]);
  }

  /* set distances to max of uint16_t (meaning no result was obtained) */
  for (i = 0; i < rall.num_nodes; i++) {
    rall.distances[i] = NO_DISTANCE;
  }
}

void prepare_rall_serial(int encoded_length, char* buffer) {
  char addr_list[5 * (rall.num_nodes - 1) + 1];
  int i;
  int addr_list_i;

  PRINTF("Preparing RALL serial (%d nodes): addr_list len %d\n", rall.num_nodes, 5 * (rall.num_nodes - 1) + 1);
  PRINTF("Preparing RALL serial (%d nodes): total len %d\n", rall.num_nodes, encoded_length);

  memset(buffer, 0, encoded_length + 1);

  addr_list_i = 0;
  for (i = 0; i < rall.num_nodes; i++) {
    if (rall.init != rall.nodes[i]) {
      snprintf(
        &addr_list[addr_list_i * 5],
        sizeof(addr_list) - (addr_list_i * 5),
        " %04X", rall.nodes[i]
      );

      PRINTF("Adding node %d: %s (remaining size %lu)\n",
        i, &addr_list[addr_list_i * 5], sizeof(addr_list) - (addr_list_i * 5));
      PRINTF("Current: %s\n", addr_list);

      addr_list_i++;
    }
#if DEBUG
    else {
      PRINTF("Skipping node %d: %04X\n", i, rall.nodes[i]);
    }
#endif
  }

  snprintf(buffer, encoded_length + 1, "RALL %04X%s\n", rall.init, addr_list);
  PRINTF("Ready (buffer): %s\n", buffer);
  PRINTF("Ready: (%d) RALL %04X%s\n", encoded_length + 1, rall.init, addr_list);
}

/* multi_range_with sends RALL init nodes to module */
int multi_range_with(node_id_t init, const node_id_t* nodes, uint8_t num_nodes) {
  node_id_t* all_nodes;
  all_nodes = NULL;

  if (state != ST_IDLE) {
    PRINTF("Err, state %d\n", state);
    return 0;
  }

  PRINTF("RALL requested with init %04X\n", init);

  /* load information in the structure for the RALL request */
  if (nodes == NULL) { // if nodes were not specified, send to all neighbors

    /* retrieve neighbors ids and this device id;
     * this group may or may not contain the initiator
     */
    num_nodes = nbr_table_all_node_ids(&all_nodes); // returns the number of neighbors +1

    PRINTF("RALL group from neighbors (%d nodes)\n", num_nodes);

    /* if the init is not in the group, increase num_nodes add it;
     * notice we search on the array of this scope, no need for a mutex;
     */
    if (index_from_node_id(init, all_nodes, num_nodes) == -1) {
      num_nodes++;
      all_nodes = (node_id_t*)realloc(all_nodes, num_nodes * sizeof(node_id_t));
      all_nodes[num_nodes - 1] = init;

      PRINTF("RALL group did not include init (%d nodes)\n", num_nodes);
    }
#if DEBUG
    else {
      PRINTF("RALL group already included init (%d nodes)\n", num_nodes);
    }
#endif

    /* check bounds for the number of nodes */
    if (num_nodes <= 1) {
      PRINTF("No nodes in RALL group (%d)\n", num_nodes);

      return 0;
    }

    prepare_rall_request(init, all_nodes, num_nodes);
    free(all_nodes);
  }
  else { // if nodes were passed to this function explicitly

    if (num_nodes == 0) {
      PRINTF("RALL group should contain at least 2 nodes\n");

      return 0;
    }

    /* copy nodes */
    all_nodes = (node_id_t*)malloc(num_nodes * sizeof(*nodes));
    memcpy(all_nodes, nodes, num_nodes * sizeof(*nodes));

    /* if the init is not in the group, increase num_nodes and add it;
     */
    if (index_from_node_id(init, all_nodes, num_nodes) == -1) {
      num_nodes++;
      all_nodes = (node_id_t*)realloc(all_nodes, num_nodes * sizeof(node_id_t));
      all_nodes[num_nodes - 1] = init;

      PRINTF("RALL group did not include init (%d nodes)\n", num_nodes);
    }
#if DEBUG
    else {
      PRINTF("RALL group already included init (%d nodes)\n", num_nodes);
    }
#endif

    /* check bounds for the number of nodes */
    if (num_nodes <= 1) {
      PRINTF("No nodes in RALL group\n");

      return 0;
    }
    if (num_nodes > MAX_NODES + 1) {
      PRINTF("Too many nodes in RALL group (%d)\n", num_nodes);

      return 0;
    }

    prepare_rall_request(init, all_nodes, num_nodes);
    free(all_nodes);
  }

  /* prepare the serial transmission for the first RALL request */
  PRINTF("Preparing RALL to serial (%d nodes)\n", num_nodes);
  int encoded_length = 4 + (5 * rall.num_nodes) + 1; // 4 for request, 5 for each node id, +1 is for \n
  char buffer[encoded_length + 1];  // +1 is for \0
  prepare_rall_serial(encoded_length, buffer);

  /* transmit RALL <initiator> <other nodes> */
  if (!send_serial(buffer, encoded_length)) {
    return 0;
  }

  state = ST_RANGING_ALL;

  return 1;
}

/* build_adjm sends RALL init nodes to module */
int build_adjm(const node_id_t* nodes, uint8_t num_nodes) {
  int i, j;
  node_id_t* all_nodes;
  all_nodes = NULL;

  if (state != ST_IDLE) {
    PRINTF("Err, state %d\n", state);
    return 0;
  }

  /* clear ADJM data */
  if (adjm.nodes != NULL) {
    free(adjm.nodes);
    adjm.nodes = NULL;
  }
  if (adjm.matrix != NULL) {
    free(adjm.matrix);
    adjm.matrix = NULL;
  }
  adjm.num_nodes = 0;

  /* store the nodes of the adjacency matrix */
  if (nodes == NULL) { // if nodes were not specified, pick all current neighbors
    adjm.num_nodes = nbr_table_all_node_ids(&all_nodes);

    /* check bounds for the number of nodes */
    if (adjm.num_nodes <= 1) {
      PRINTF("No nodes in ADJM group\n");

      return 0;
    }
  }
  else { // if nodes were specified in the call

    /* copy nodes */
    all_nodes = (node_id_t*)malloc(num_nodes * sizeof(*nodes));
    memcpy(all_nodes, nodes, num_nodes * sizeof(*nodes));

    adjm.num_nodes = num_nodes;

    /* check bounds for the number of nodes */
    if (adjm.num_nodes <= 1) {
      PRINTF("No nodes in ADJM group\n");

      return 0;
    }
    if (adjm.num_nodes > MAX_NODES + 1) {
      PRINTF("Too many nodes in ADJM group (%d)\n", num_nodes);

      return 0;
    }
  }

  /* assign the nodes for this procedure */
  adjm.nodes = all_nodes;

  /* allocate the matrix */
  adjm.matrix = (adjm_cell_t *)malloc(adjm.num_nodes * adjm.num_nodes * sizeof(adjm_cell_t));

  /* initialize matrix with max of uint16_t (meaning no result was obtained) */
  for (i = 0; i < adjm.num_nodes; i++) {
    for (j = 0; j < adjm.num_nodes; j++) {
      adjm.matrix[i * adjm.num_nodes + j].distance = NO_DISTANCE;
    }
  }

  /* load information in the structure for the RALL request */
  current_rall_init_index = 0;
  prepare_rall_request(adjm.nodes[current_rall_init_index], adjm.nodes, adjm.num_nodes);

  PRINTF("--- ADJM REQUEST ---\n");
  PRINTF("\tFirst init: %04X\n", adjm.nodes[current_rall_init_index]);
#if DEBUG
  for (i = 0; i < adjm.num_nodes; i++) {
    PRINTF("\tAdjm->nodes[%d] = %04X\n", i, adjm.nodes[i]);
  }
#endif

  /* prepare the serial transmission for the first RALL request */
  int encoded_length = 4 + (5 * adjm.num_nodes) + 1; // 4 for request, 5 for each node id, +1 is for \n
  char buffer[encoded_length + 1];  // +1 is for \0
  current_rall_init_index = 0;
  prepare_rall_serial(encoded_length, buffer);

  /* transmit RALL <initiator> <other nodes> */
  if (!send_serial(buffer, encoded_length)) {
    return 0;
  }

  state = ST_ADJM;

  return 1;
}

int pop_recv_packet(uint8_t* buf, node_id_t* from) {
  int res = 0;
  pthread_mutex_lock(&recv_mutex);
  if (recv_qlen > 0) {
    memcpy(buf, recv_buf, recv_len);
    recv_qlen = 0;
    *from = recv_src_id;
    res = recv_len;
  }
  pthread_mutex_unlock(&recv_mutex);
  return res;
}

int index_from_node_id(node_id_t node_id, const node_id_t* node_ids, uint8_t num_nodes) {
  int i;

  //node_id = ntohs(node_id);

  for (i = 0; i < num_nodes; i++) {
    if (node_ids[i] == node_id)
      return i;
  }
  return -1;
}

void register_sent_cb(sent_cb_t cb) {
  sent_callback = cb;
}

void register_recv_cb(recv_cb_t cb) {
  recv_callback = cb;
}

void register_rng_cb(rng_cb_t cb) {
  rng_callback = cb;
}

void register_nbr_cb(nbr_cb_t cb) {
  nbr_callback = cb;
}

void register_rall_cb(rall_cb_t cb) {
  rall_callback = cb;
}

void register_adjm_cb(adjm_cb_t cb) {
  adjm_callback = cb;
}

node_id_t get_node_id(void) {
  return device_id;
}

int power_off() {
  sleep(1);

  /* flush serial data (both directions) */
  usleep(10000); // required for USB serial flush
  tcflush(serial_fd,TCIOFLUSH);

  /* request to reset radio and turn it OFF */
  char request[6];
  snprintf(request, 5, "POFF");
  request[4] = '\n';
  request[5] = '\0';
  if (!send_serial(request, 5))
    return 0;
  return 1;
}

int power_on() {
  sleep(1);

  /* flush serial data (both directions) */
  usleep(10000); // required for USB serial flush
  tcflush(serial_fd,TCIOFLUSH);

  /* request to reset radio and turn it OFF */
  char request[6];
  snprintf(request, 5, "PON_");
  request[4] = '\n';
  request[5] = '\0';
  if (!send_serial(request, 5))
    return 0;
  return 1;
}
