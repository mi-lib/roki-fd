include ../config

ROOTDIR=..
LIBDIR=$(ROOTDIR)/lib
INCLUDE=-I$(ROOTDIR)/include -I$(PREFIX)/include

CC=gcc
CFLAGS=-ansi -Wall -fPIC -O3 $(INCLUDE) -funroll-loops
LD=gcc
LDFLAGS=-shared
SIGNUP=echo "Roki-FD ver."$(VERSION)" Copyright (C) 2018 Tomomichi Sugihara (Zhidao)" >>

OBJ=
DLIB=librokifd.so

TARGET=$(DLIB)

all: $(TARGET)

$(DLIB): $(OBJ)
	@echo " LD	" $^
	@$(LD) $(LDFLAGS) -o $@ $^ > /dev/null; $(SIGNUP) $@
	-@mv $@ $(LIBDIR)
%.o: %.c
	@echo " CC	" $<
	@$(CC) $(CFLAGS) -c $< > /dev/null
clean:
	@echo " CLEAN"
	-@rm -f *.o *~ \#* $(TARGET)
