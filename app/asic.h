#ifndef __ASIC_H__
#define __ASIC_H__

#include "periph/i2c.h"
#include <xtimer.h>

#define ENABLE_DEBUG    (0)
#include "debug.h"

#ifdef ROOM_TYPE
#define BUILD 210
#define NUMASICS 4
#elif defined(DUCT_TYPE)
#define BUILD 215
#define NUMASICS 4
#elif defined(DUCT6_TYPE)
#define BUILD 217
#define NUMASICS 6
#endif


typedef struct {
  i2c_t   pex_i2c;
  i2c_t   asic_i2c;
  uint8_t addr [6];
  uint8_t shadowHA;
  uint8_t shadowLA;
  uint8_t shadowHB;
  uint8_t shadowLB;
  uint16_t calres [6];
  uint16_t cal_pulse;
} asic_tetra_t;

typedef struct {
  uint64_t uptime;
  uint8_t sampledata [4][70];
  uint8_t primary;
} measurement_t;

int8_t asic_init(asic_tetra_t *a, i2c_t pex_i2c, i2c_t asic_i2c);
int8_t asic_program(asic_tetra_t *a, uint8_t num);
int8_t asic_led(asic_tetra_t *a, uint8_t red, uint8_t green, uint8_t blue);
int8_t asic_measure(asic_tetra_t *a, uint8_t primary, measurement_t *m);
int8_t asic_measure_just_iq(asic_tetra_t *a, uint8_t primary, measurement_t *m);
int8_t asic_fake_measure(asic_tetra_t *a);
int8_t asic_calibrate(asic_tetra_t *a);
int8_t asic_check_ready(asic_tetra_t *a, uint8_t num);
int8_t asic_all_out_of_reset(asic_tetra_t *a);
int8_t asic_configure(asic_tetra_t *a, uint8_t num);
#endif
