#pragma once

#include "common.h"

#define NODE_1 0x45bb
#define NODE_2 0x4890
#define NODE_3 0x0C16
#define NODE_4 0x4829

#define DRONE_ALTITUDE 0
#define UWB_MODULE_COUNTER 5

extern uint16_t distances[6];
extern double vw_uwb_1[2];
extern double vw_uwb_2[2];
extern double vw_uwb_3[2];
extern double vw_uwb_4[3];

#ifdef __cplusplus
extern "C" {
#endif
  void initialize_UWB(void);
  bool read_UWB(void);
  void write_UWB(double, double);
  void *uwb_f(void *);
  void node_index_from_id(unsigned int);
  double *get_UWB_data(void);
  uint16_t *get_UWB_dist(void);

  void compute_coordinates(void);
  void sig_handler(void);

#ifdef __cplusplus
}
#endif
