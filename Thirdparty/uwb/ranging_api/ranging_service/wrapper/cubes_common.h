#ifndef CUBES_COMMON_H
#define CUBES_COMMON_H
#include <stdint.h>

/* Max number of nodes in the neighbors table;
 * MAX_NODES + 1 is the supported number of nodes in the system.
 */
#define MAX_NODES 3

int decode_hex(const char* input, uint16_t len, char* decoded);
int encode_hex(const char* input, uint16_t len, char* encoded);

#define htons(A) ((((uint16_t)(A) & 0xff00) >> 8) | \
                  (((uint16_t)(A) & 0x00ff) << 8))
#define htonl(A) ((((uint32_t)(A) & 0xff000000) >> 24) | \
                  (((uint32_t)(A) & 0x00ff0000) >> 8)  | \
                  (((uint32_t)(A) & 0x0000ff00) << 8)  | \
                  (((uint32_t)(A) & 0x000000ff) << 24))

#define ntohs  htons
#define ntohl  htonl


#endif
