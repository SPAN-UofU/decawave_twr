# Stuff based on the environment. This assumes stuff to be compiled on the
# beaglebone and the am335x_pru_package checked out.
# https://github.com/beagleboard/am335x_pru_package

# In case you cross compile this on a different architecture, uncomment this
# and set the prefix
#CROSS_COMPILE?=arm-arago-linux-gnueabi-
CROSS_COMPILE?=arm-linux-gnueabihf-

# Tuning options for ARM CPU.
#ARM_OPTIONS?=-mtune=cortex-a8 -march=armv7-a

# Location of am335x package https://github.com/beagleboard/am335x_pru_package
#AM335_BASE=~/am335x_pru_package/pru_sw
#PASM=$(AM335_BASE)/utils/pasm
#LIBDIR_APP_LOADER?=$(AM335_BASE)/app_loader/lib
#INCDIR_APP_LOADER?=$(AM335_BASE)/app_loader/include

PASM=pasm
LIBDIR_APP_LOADER?=/usr/lib
INCDIR_APP_LOADER?=/usr/include

CFLAGS+= -Wall -I$(INCDIR_APP_LOADER) -std=c99 -D_XOPEN_SOURCE=500 -O2 $(ARM_OPTIONS)
LDFLAGS+=-lpthread -lm
PRUSS_LIBS=-Wl,-rpath=$(LIBDIR_APP_LOADER) -L$(LIBDIR_APP_LOADER) -lprussdrv

dw1000-objs := platform.o deca_device.o deca_params_init.o
cc1200-objs := cc1200.o

all: clean SPI_bin.h dw1000_mdrfs dw1000_rfs

clean:
	rm -f cc1200_xo_sync cc1200_msg cc1200_app dw1000_ref dw1000_sync dw1000_rfs dw1000_atwr dw1000_ds_twr testclk *.o *_bin.h

SPI_bin.h: SPI.p
	$(PASM) -V3 -c $<

cc1200_xo_sync: cc1200_xo_sync.o
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(PRUSS_LIBS) $(LDFLAGS)
	
cc1200_msg: cc1200_msg.o
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

cc1200_app: cc1200_app.o $(cc1200-objs)
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

dw1000_ref: dw1000_ref.o $(dw1000-objs)
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

dw1000_sync: dw1000_sync.o $(dw1000-objs)
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

dw1000_rfs: dw1000_rfs.o $(dw1000-objs) $(cc1200-objs)
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(PRUSS_LIBS) $(LDFLAGS)

dw1000_mdrfs: dw1000_mdrfs.o $(dw1000-objs) $(cc1200-objs)
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(PRUSS_LIBS) $(LDFLAGS)

dw1000_atwr: dw1000_atwr.o $(dw1000-objs) $(cc1200-objs)
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

dw1000_ds_twr: dw1000_ds_twr.o $(dw1000-objs)
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

testclk: testclk.o
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)
