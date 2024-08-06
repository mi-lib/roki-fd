INCLUDE=`roki-fd-config -I`
LIB=`roki-fd-config -L`
LINK=`roki-fd-config -l`

TARGET=$(shell ls *.c | xargs -i basename {} .c | tr -s "\n" " ")

CC=gcc
CFLAGS=-ansi -Wall -O3 $(LIB) $(INCLUDE) -funroll-loops -g

all: $(TARGET)
%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LINK)
clean :
	-@rm -f *.o *~ core *test $(TARGET)
