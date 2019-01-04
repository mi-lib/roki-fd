include ../../config

INCLUDE=`rokifd-config -I`
LIB=`rokifd-config -L`
LINK=`rokifd-config -l`

CC=gcc
CFLAGS=-ansi -Wall -O3 $(LIB) $(INCLUDE) -funroll-loops -g

%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LINK)
clean :
	rm -f *.o *~ core *test
