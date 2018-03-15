APPLICATION = anemometer_qfw_tcp
RIOTBASE = $(CURDIR)/../RIOT-OS
BOARD = hamilton

CFLAGS += -DMODEL_3C
#CFLAGS += -DROOM_TYPE
#CFLAGS += -DDUCT_TYPE
CFLAGS += -DDUCT6_TYPE
CFLAGS += -DAUTORANGE

PROGRAMMER ?= jlink

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

USEPKG += openthread
OPENTHREAD_TYPE ?= ftd
ifeq ($(OPENTHREAD_TYPE),mtd)
    # MTD: A Minimal Thread Device does not have router functionality
    # compiled in. As a result, it is not necessary to configure the
    # routerrole on an MTD. At the same time, an MTD may or may not be sleepy.
    USEMODULE += openthread-mtd
    USEMODULE += openthread-cli-mtd
    BOARD ?= hamilton
else
    # ftd: A Full Thread Device has router functionality compiled in
    USEMODULE += openthread-ftd
    USEMODULE += openthread-cli-ftd
    #BOARD ?= samr21-xpro
    BOARD ?= hamilton
    CFLAGS += -DCLOCK_USE_PLL=1
endif

#Define PANID, CHANNEL, TXPOWER, and UART_BAUDRATE used by default
OPENTHREAD_PANID ?= 0xbeef
OPENTHREAD_CHANNEL ?= 24
OPENTHREAD_TXPOWER ?= 4
OPENTHREAD_UART_BAUDRATE ?= 115200

CFLAGS += -DOPENTHREAD_PANID=${OPENTHREAD_PANID}
CFLAGS += -DOPENTHREAD_CHANNEL=${OPENTHREAD_CHANNEL}
CFLAGS += -DOPENTHREAD_TXPOWER=${OPENTHREAD_TXPOWER}
CFLAGS += -DOPENTHREAD_UART_BAUDRATE=${OPENTHREAD_UART_BAUDRATE}
CFLAGS += -DOPENTHREAD_SOURCE=${HOSTNAME}
CFLAGS += -DDEVELHELP
#CFLAGS += -DHIGH_DATA_RATE=1

#CFLAGS += -DINSTRUMENT_TCP

ifneq (,$(filter hamilton samr21-xpro,$(BOARD)))
  DRIVER := at86rf233
  CFLAGS    += -DAUTO_CSMA_EN=1
endif
ifneq (,$(filter iotlab-m3 fox iotlab-a8-m3,$(BOARD)))
  DRIVER := at86rf231
endif

ifneq (,$(filter at86rf2%,$(DRIVER)))
  FEATURES_REQUIRED += periph_spi periph_gpio
endif

USEMODULE += $(DRIVER)
USEMODULE += schedstatistics

# CPU clock speed
CFLAGS    += -DCLOCK_USE_ADAPTIVE=0
CFLAGS    += -DSENSOR_COMPUTOFFLOAD=1
CFLAGS    += -DDMAC_ENABLE=0
USEMODULE += task_sched
USEMODULE += tcp_freebsd
USEMODULE += tcp_freebsd_internal
USEMODULE += tcp_freebsd_internal_cc
USEMODULE += ipv6_addr
CFLAGS += -DSOCK_HAS_IPV6
USEMODULE += posix_sockets
USEMODULE += sock_tcp_freebsd

USEMODULE += xtimer
USEMODULE += random
USEMODULE += ps

USEMODULE += rtt_stdio
#CFLAGS    += -DRTT_STDIO_DISABLE_STDIN
#CFLAGS 		+= -DSTDIO_POLL_INTERVAL=5000U
#CFLAGS    += -DRTT_STDIO_ENABLE_BLOCKING_STDOUT
CFLAGS    += -DTHREAD_STACKSIZE_DEFAULT=1670
CFLAGS    += -DTHREAD_STACKSIZE_MAIN=1670
# CPU clock speed
#CFLAGS    += -DCLOCK_USE_OSCULP32_DFLL=1
CFLAGS    += -DCLOCK_USE_ADAPTIVE=0
CFLAGS    += -DDMAC_ENABLE=0
CFLAGS    += -DVDD=1800

# Sensors
USEMODULE += saul_reg        # SAUL: sensor/actuator abstraction layer
USEMODULE += auto_init_saul
#USEMODULE += tmp006          # ambient temperature
# USEMODULE += hdc1000         # humidity and temperature
USEMODULE += fxos8700        # acceleration and magnetic field
# USEMODULE += apds9007        # illumination
# USEMODULE += ekmb            # pir-based occupancy
# USEMODULE += pulse_counter   # simple push button

# Security
USEMODULE += crypto
USEMODULE += cipher_modes
CFLAGS    += -DCRYPTO_AES

# Radio
# USEMODULE += at86rf2xx
# CFLAGS    += -DIEEE802154_DEFAULT_CHANNEL=25
# CFLAGS    += -DAUTO_CSMA_EN=1

QUIET ?= 1

#FEATURES_REQUIRED += periph_timer

#required for C++ compiling
CXXEXFLAGS += -fno-rtti
USEMODULE += cpp11-compat

include $(RIOTBASE)/Makefile.include
