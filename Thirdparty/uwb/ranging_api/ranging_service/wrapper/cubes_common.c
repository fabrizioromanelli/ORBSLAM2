#include "cubes_common.h"

#define HEX2N(h) (((h)>='0' && (h)<='9') ? ((h) - '0') : \
                        ( (h>='A' && (h)<='F') ? (10 + (h)-'A') : 255) \
                 )

#define N2HEX(n) ((n)<10 ? (n) + '0' : (n) + 'A' - 10)

int decode_hex(const char* input, uint16_t len, char* decoded) {
  if (len % 2)
    return 0;
  
  uint16_t out_len = len/2;

  for (int i=0; i<out_len; i++) {
    char c1 = input[i*2]; 
    char c2 = input[i*2 + 1];
    uint8_t n = HEX2N(c1);
    if (n == 255)
      return 0;
    decoded[i] = n<<4;
    n = HEX2N(c2);
    if (n == 255)
      return 0;
    decoded[i] |= n;
  }

  return out_len;
}

int encode_hex(const char* input, uint16_t len, char* encoded) {
  uint16_t out_len = len*2;

  for (int i=0; i<len; i++) {
    encoded[i*2]   = N2HEX(((uint8_t)input[i]) >> 4); 
    encoded[i*2+1] = N2HEX(((uint8_t)input[i]) & 0x0F); 
  }

  return out_len;
}

