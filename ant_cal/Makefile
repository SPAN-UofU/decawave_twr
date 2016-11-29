CFLAGS+= -Wall -I$(INCDIR_APP_LOADER) -std=c99 -D_XOPEN_SOURCE=500 -O2 $(ARM_OPTIONS)
LDFLAGS+=-lpthread -lm

dw1000-objs := platform.o deca_device.o deca_params_init.o deca_range_tables.o

all: dw1000_init dw1000_resp

clean:
	rm -f dw1000_init dw1000_resp *.o

dw1000_tx: dw1000_init.o $(dw1000-objs)
	gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

dw1000_rx: dw1000_resp.o $(dw1000-objs)
	gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

