CC := gcc
RM := rm -f

LUA := luajit

CFLAGS ?= -O2
override CFLAGS += -D_FORTIFY_SOURCE=2 `pkg-config $(LUA) --cflags` -std=gnu11 -pipe -fpic -shared -fno-plt -fexceptions -fasynchronous-unwind-tables -fno-strict-aliasing -fno-strict-overflow -fwrapv -fstack-clash-protection -fstack-protector-strong -fcf-protection=full --param ssp-buffer-size=4 -Wall -Wextra -Werror -Wshadow -Wformat -Wformat-security

LDFLAGS ?= -Wl,-z,relro -Wl,-z,now -Wl,-z,defs -Wl,-s
override LDFLAGS += -shared -Wl,`pkg-config $(LUA) --libs`

all: geohash.so

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

geohash.o:

geohash.so: geohash.o
	$(CC) $(LDFLAGS) -o $@ $^

clean:
	$(RM) *.o *.so
