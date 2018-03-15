#include <stdint.h>
#include <thread.h>

typedef struct __attribute__((packed))
{
  uint8_t   l7type;     // 0
  uint8_t   type;       // 1
  uint16_t  seqno;      // 2:3
  uint8_t   primary;    // 4
  uint8_t   buildnum;   // 5
  int16_t   acc_x;      // 6:7
  int16_t   acc_y;      // 8:9
  int16_t   acc_z;      // 10:11
  int16_t   mag_x;      // 12:13
  int16_t   mag_y;      // 14:15
  int16_t   mag_z;      // 16:17
  uint16_t  temp;        // 18:19
  int16_t   reserved;    // 20:21
  uint8_t   max_index[3]; // 22:24
  uint8_t   parity;    // 25
  uint16_t  cal_res;   // 26:27
  //Packed IQ data for 4 pairs
  //M-3, M-2, M-1, M
  uint8_t data[3][16];  //28:75
  uint16_t tof_sf[3];   //76:81
} measure_set_t; //82 bytes

kernel_pid_t start_sendloop(void);

#define SENDER_PORT 49999
#define RECEIVER_IP "fdde:ad00:beef:0:e9f0:45bc:c507:6f0e"
#define RECEIVER_PORT 50000
