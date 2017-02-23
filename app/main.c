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

#include <at30ts74.h>
#include <mma7660.h>
#include <periph/gpio.h>
#include "asic.h"
#include "version.h"
#include <reboot.h>
// 1 second, defined in us
#define INTERVAL (1000000U)
#define NETWORK_RTT_US 1000000
#define COUNT_TX (-4)
#define A_LONG_TIME 5000000U
#define MAIN_QUEUE_SIZE     (8)
#define L7_TYPE 7
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

uint16_t ms_seqno = 0;

#define XOR_OFFSET 5

typedef struct __attribute__((packed))
{
  uint16_t l7type;
  uint8_t type;
  uint16_t seqno;
  uint16_t build;
  uint16_t cal_pulse;
  uint16_t calres[4];
  uint64_t uptime;
  uint8_t primary;
  uint8_t data[4][70];
} measure_set_t;

#define MSI_MAX 4
uint8_t msi;
measure_set_t msz[MSI_MAX];
uint8_t xorbuf [sizeof(measure_set_t)];
extern void send_udp(char *addr_str, uint16_t port, uint8_t *data, uint16_t datalen);

void tx_measure(asic_tetra_t *a, measurement_t *m)
{
  msi++;
  msi &= (MSI_MAX-1);
  msz[msi].seqno = ms_seqno++;
  msz[msi].build = BUILD_NUMBER;
  msz[msi].cal_pulse = a->cal_pulse;
  for (int i = 0; i < 4; i++)
  {
    msz[msi].calres[i] = a->calres[i];
  }
  memcpy(&(msz[msi].data[0]), &(m->sampledata[0]), 4*70);
  msz[msi].uptime = m->uptime;
  msz[msi].primary = m->primary;
  msz[msi].type = msi+10;
  msz[msi].l7type = L7_TYPE;
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
  xorbuf[2] = 0x55;
  send_udp("ff02::1",4747,xorbuf,sizeof(measure_set_t));
}
void initial_program(asic_tetra_t *a)
{
  int8_t e;
  e = (int)asic_init(a, I2C_0);
  asic_led(a, 1,1,0);
  printf("[init] first errcode was %d\n", e);
  uint8_t bad = 0;
  for (int i = 0; i < 4; i ++)
  {
    e = asic_program(a, i);
    printf("[init] program pass 1 for %d code was %d\n", i, e);
  }
  for (int i = 0; i < 4; i ++)
  {
    e = asic_program(a, i);
    if (e) bad = 1;
    printf("[init] program pass 2 for %d code was %d\n", i, e);
  }
  asic_led(a, 1,0,1);
  xtimer_usleep(100000); //100ms
  for (int i = 0; i < 4; i ++)
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
  for (int i = 0; i < 4; i ++)
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
    //for (int8_t i = 0; i < 4; i++)
    //{
    //  printf("[cal] ASIC %d measured %d\n", i, a.calres[i]);
    //}

    for (int i = 0; i < 128; i++)
    {
      e = asic_fake_measure(&a);
      if (e) goto failure;
      for (int p = 0; p < 4; p ++)
      {
        sampm[p].uptime = xtimer_usec_from_ticks64(xtimer_now64());
        e = asic_measure(&a, p, &sampm[p]);
        if(e) goto failure;
        //dump_measurement(&a, &m);
      }
      for (int p = 0; p < 4; p ++)
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
    printf("[init] booting build b%d\n",BUILD_NUMBER);
    begin();

    return 0;
}
