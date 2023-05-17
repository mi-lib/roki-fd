include ../../config

INCLUDE=`roki_fd-config -I`
LIB=`roki_fd-config -L`
LINK=`roki_fd-config -l`

CC=gcc
CFLAGS=-ansi -Wall -O3 $(LIB) $(INCLUDE) -funroll-loops -g

%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LINK)
clean :
	rm -f *.o *~ core *test
