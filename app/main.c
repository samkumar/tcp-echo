#include <stdio.h>
#include <rtt_stdio.h>
#include "shell.h"
#include "thread.h"
#include "xtimer.h"
#include <string.h>
#include "msg.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/netreg.h"
#include "phydat.h"
#include "saul_reg.h"
#include <periph/gpio.h>
#include "asic.h"

// 1 second, defined in us
#define INTERVAL (1000000U)
#define NETWORK_RTT_US 1000000
#define COUNT_TX (-4)
#define A_LONG_TIME 5000000U
#define MAIN_QUEUE_SIZE     (8)

#define DIRECT_DATA_ADDRESS 0x65

//Anemometer v2
#define L7_TYPE 9

static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

uint16_t ms_seqno = 0;

#define XOR_OFFSET 4

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
  int16_t   hdc_temp;   // 18:19
  int16_t   hdc_hum;    // 20:21
  uint8_t   max_index[3]; // 22:24
  uint8_t   parity;    // 25
  uint16_t  cal_res;   // 26:27
  //Packed IQ data for 4 pairs
  //M-3, M-2, M-1, M
  uint8_t data[3][16];  //28:75
} measure_set_t; //76 bytes

// typedef struct __attribute__((packed))
// {
//   uint16_t l7type;
//   uint8_t type;
//   uint16_t seqno;
//   uint16_t build;
//   uint16_t cal_pulse;
//   uint16_t calres[4];
//   uint64_t uptime;
//   uint8_t primary;
//   uint8_t data[4][70];
// } measure_set_t;

#define MSI_MAX 4
uint8_t msi;
measure_set_t msz[MSI_MAX];
uint8_t xorbuf [sizeof(measure_set_t)];
extern void send_udp(char *addr_str, uint16_t port, uint8_t *data, uint16_t datalen);

void reboot(void){
  NVIC_SystemReset();
}
//In a 64 byte array of 16 big endian int16_t pairs of I and Q, what
//is the index where the complex magnitude is greatest [0,15]?
uint8_t calculate_max_index(uint8_t *data, uint8_t print) {
  int64_t ix;
  int64_t qx;
  uint64_t magsqr;
  uint64_t magsqrmax;
  uint8_t indexmax;

  magsqrmax = 0;
  indexmax = 0;

  for (int i = 0; i < 16; i++) {
    qx = (int16_t) ((uint16_t)data[i<<2] + (((uint16_t)data[(i<<2) + 1]) << 8));
    ix = (int16_t) ((uint16_t)data[(i<<2) + 2] + (((uint16_t)data[(i<<2) + 3]) << 8));
    magsqr = (qx * qx) + (ix * ix);
    if (print) {
      printf("%8lx ", (long unsigned int)magsqr);
    }
    if (magsqr > magsqrmax) {
      indexmax = i;
      magsqrmax = magsqr;
    }
  }
  return indexmax;
}


saul_reg_t *sensor_temp_t    = NULL;
saul_reg_t *sensor_hum_t     = NULL;
saul_reg_t *sensor_mag_t     = NULL;
saul_reg_t *sensor_accel_t   = NULL;


void sensor_config(void) {

    sensor_mag_t     = saul_reg_find_type(SAUL_SENSE_MAG);
    if (sensor_mag_t == NULL) {
		  DEBUG("[ERROR] Failed to init MAGNETIC sensor\n");
  	} else {
  		DEBUG("MAGNETIC sensor OK\n");
  	}

    sensor_accel_t   = saul_reg_find_type(SAUL_SENSE_ACCEL);
    if (sensor_accel_t == NULL) {
		  DEBUG("[ERROR] Failed to init ACCEL sensor\n");
  	} else {
  		DEBUG("ACCEL sensor OK\n");
  	}

}

void tx_measure(asic_tetra_t *a, measurement_t *m)
{
  phydat_t output; /* Sensor output data (maximum 3-dimension)*/
  int dim;         /* Demension of sensor output */
  uint8_t parity;
  msi++;
  parity = 0;
  msi &= (MSI_MAX-1);
  msz[msi].l7type = L7_TYPE;
  msz[msi].type = 0;
  msz[msi].buildnum = BUILD;
  msz[msi].primary = m->primary;
  msz[msi].seqno = ms_seqno++;
  #if defined(DUCT6_TYPE)
  if (m->primary == 5) {
    ms_seqno += 2; //for 6 channel skip seqno % 8 == 6/7
  }
  #endif
  msz[msi].parity = 0;

  /* Magnetic field 3-dim */
  dim = saul_reg_read(sensor_mag_t, &output);
  if (dim > 0) {
      msz[msi].mag_x = output.val[0]; msz[msi].mag_y = output.val[1]; msz[msi].mag_z = output.val[2];
  } else {
      DEBUG("[ERROR] Failed to read magnetic field\n");
  }

  /* Acceleration 3-dim */
  dim = saul_reg_read(sensor_accel_t, &output);
  if (dim > 0) {
      msz[msi].acc_x = output.val[0]; msz[msi].acc_y = output.val[1]; msz[msi].acc_z = output.val[2];
  } else {
      printf("[ERROR] Failed to read Acceleration\n");
  }

  for (int i = 0; i < 6; i++) {
    printf("calred %d = %d\n", i, a->calres[i]);
  }
  //
  // for (int i = 0; i < 16; i++) {
  //     printf("%8u ", i);
  // }
  // printf(" =======\n");
  for(int i = 0;i<3;i++) {
    uint8_t maxindex = calculate_max_index(m->sampledata[i], 0);
    //printf(" p=%d m[%d] = %d\n", m->primary, i, maxindex);
    msz[msi].max_index[i] = maxindex;
    if (maxindex <= 3) {
      maxindex = 0;
    } else {
      maxindex -= 3;
    }
    //Copy 4 IQ pairs starting from 3 before the max index, unless the max is right
    //at the beginning, then start from 0
    memcpy(&(msz[msi].data[i][0]), &(m->sampledata[i][maxindex<<2]), 16);
  }

  //Send the calibration result for the primary
  msz[msi].cal_res = a->calres[m->primary];
  msz[msi].type = msi+10;

  //Calculate the parity
  for (int i = 0; i < sizeof(measure_set_t); i++) {
    parity ^= ((uint8_t*)&msz[msi])[i];
  }
  msz[msi].parity = parity;

  send_udp("ff02::1",4747,(uint8_t*)&(msz[msi]),sizeof(measure_set_t));

  //Clear body of xor message
  memset(xorbuf+XOR_OFFSET, 0, sizeof(measure_set_t)-XOR_OFFSET);
  //Copy the header (L7Type and seqno)
  memcpy(xorbuf, (uint8_t*)&msz[msi], XOR_OFFSET);
  for(int i = 0; i < MSI_MAX; i++)
  {
    uint8_t *paybuf = ((uint8_t*)&msz[i]);
    for (int k = XOR_OFFSET; k < sizeof(measure_set_t);k++)
    {
      xorbuf[k] ^= paybuf[k];
    }
  }
  //Set the type field to 0x55;
  xorbuf[1] = 0x55;
  send_udp("ff02::1",4747,xorbuf,sizeof(measure_set_t));

  //Also write the packet to I2C_0
  i2c_write_bytes(I2C_0, DIRECT_DATA_ADDRESS, "cafebabe",8);
  i2c_write_bytes(I2C_0, DIRECT_DATA_ADDRESS, (uint8_t*)&(msz[msi]),sizeof(measure_set_t));
  for (int i = 0; i < 3; i++) {
    i2c_write_bytes(I2C_0, DIRECT_DATA_ADDRESS, (uint8_t*)&(m->sampledata[i][0]), 64),
  }
}
void initial_program(asic_tetra_t *a)
{
  int8_t e;
  e = (int)asic_init(a, I2C_0);
  printf("[init] asic_init return with %d\n", e);

  asic_led(a, 1, 1, 1);
  printf("[init] first errcode was %d\n", e);
  uint8_t bad = 0;
  for (int i = 0; i < NUMASICS; i ++)
  {
    e = asic_program(a, i);
    printf("[init] program pass 1 for %d code was %d\n", i, e);
  }
  for (int i = 0; i < NUMASICS; i ++)
  {
    e = asic_program(a, i);
    if (e) bad = 1;
    printf("[init] program pass 2 for %d code was %d\n", i, e);
  }
  asic_led(a, 1,0,1);
  xtimer_usleep(100000); //100ms
  for (int i = 0; i < NUMASICS; i ++)
  {
    e = asic_configure(a, i);
    if (e) bad = 1;
    printf("[init] configure for %d code was %d\n", i, e);
  }
  if (bad) {
    asic_led(a, 1,0,0);
    xtimer_usleep(A_LONG_TIME);
    reboot();
  }
  asic_led(a, 1,1,1);
  xtimer_usleep(100000); //100ms
  asic_all_out_of_reset(a);
  xtimer_usleep(100000); //100ms
  bad = 0;
  for (int i = 0; i < NUMASICS; i ++)
  {
    e = asic_check_ready(a, i);
    if (e) bad = 1;
    printf("[init] %d ready was %d\n", i, e);
  }
  if (bad) {
    asic_led(a, 1,0,0);
    xtimer_usleep(A_LONG_TIME);
    reboot();
  }
  asic_led(a, 0,1,0);
  xtimer_usleep(100000); //100ms
}
#if 0
void dump_measurement(asic_tetra_t *a, measurement_t *m)
{
  printf("primary: %d\n", m->primary);
  for (int8_t num = 0; num < 4; num ++)
  {
    int16_t iz[16];
    int16_t qz[16];
    uint64_t magsqr[16];
    uint64_t magmax = 0;
    uint16_t tof_sf;
    uint8_t *b = &m->sampledata[num][0];
    tof_sf = b[0] + (((uint16_t)b[1]) << 8);
    for (int i = 0; i < 16; i++)
    {
      qz[i] = (int16_t) (b[6+i*4] + (((uint16_t)b[6+ i*4 + 1]) << 8));
      iz[i] = (int16_t) (b[6+i*4 + 2] + (((uint16_t)b[6+ i*4 + 3]) << 8));
      magsqr[i] = (uint64_t)(((int64_t)qz[i])*((int64_t)qz[i]) + ((int64_t)iz[i])*((int64_t)iz[i]));
      if (magsqr[i] > magmax)
      {
        magmax = magsqr[i];
      }
    }
    //Now we know the max, find the first index to be greater than half max
    uint64_t quarter = magmax >> 2;
    int ei = 0;
    int si = 0;
    for (int i = 0; i < 16; i++)
    {
      if (magsqr[i] < quarter)
      {
        si = i;
      }
      if (magsqr[i] > quarter)
      {
        ei = i;
        break;
      }
    }
    double s = sqrt((double)magsqr[si]);
    double e = sqrt((double)magsqr[ei]);
    double h = sqrt((double)quarter);
    double freq = tof_sf/2048.0*a->calres[num]/a->cal_pulse;
    double count = si + (h - s)/(e - s);
    double tof = (count + COUNT_TX) / freq * 8;

    //Now "linearly" interpolate
    printf("count %d /1000\n", (int)(count*1000));
    printf("tof_sf %d\n", tof_sf);
    printf("freq %d uHz\n", (int)(freq*1000));
    printf("tof %d uS\n", (int)(tof*1000));
    printf("tof 50us estimate %duS\n", (int)(count*50));
    for (int i = 0; i < 16; i++)
    {
      printf("data %d = %d + %di\n", i, qz[i], iz[i]);
    }
    printf(".\n");
  }
}
#endif
measurement_t sampm[4];
void begin(void)
{
  sensor_config();
  asic_tetra_t a;
  int8_t e;
  initial_program(&a);
  xtimer_usleep(100000);
  e = asic_calibrate(&a);
  if (e) {
    printf("calibrate failed\n");
    goto failure;
  }
  while (1)
  {
    for (int i = 0; i < 128; i++)
    {
      e = asic_fake_measure(&a);
      if (e) goto failure;
      for (int p = 0; p < NUMASICS; p ++)
      {
        e = asic_measure_just_iq(&a, p, &sampm[p]);
        if(e) goto failure;
      }
      for (int p = 0; p < NUMASICS; p ++)
      {
        tx_measure(&a, &sampm[p]);
      }
      xtimer_usleep(50000);
    }
  }
failure:
  asic_led(&a, 1,1,0);
  printf("[run] encountered failure\n");
  xtimer_usleep(A_LONG_TIME);
  reboot();
}
int main(void)
{
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    printf("[init] booting build b%d\n",BUILD);
    begin();

    return 0;
}
