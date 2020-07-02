include ../config

ROOTDIR=..
LIBDIR=$(ROOTDIR)/lib
INCLUDE=-I$(ROOTDIR)/include -I$(PREFIX)/include

CC=gcc
CFLAGS=-ansi -Wall -Werror -fPIC -O3 $(INCLUDE) -funroll-loops
LD=gcc
LDFLAGS=-shared
SIGNUP=echo "RoKi-FD ver."$(VERSION)" Copyright (C) 2018 Tomomichi Sugihara (Zhidao)" >>

OBJ=rkfd_property.o\
	rkfd_cd.o\
	rkfd_solver.o\
	rkfd_util.o\
	rkfd_sim.o\
	rkfd_penalty.o\
	rkfd_vert.o rkfd_volume.o rkfd_mlcp.o
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
