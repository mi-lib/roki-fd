# default suffices
# .z3d : 3D shape model
# .zkc : kinematic chain model
# .zkci: initial state of a kinematic chain
# .zvs : vector sequence
# .zkcs: kinematic chain configuration sequence
# .zef : external force info.

include ../config

INCLUDE=-I. -I../include -I$(PREFIX)/include
LIB=-L../lib -L$(PREFIX)/lib
BINDIR=$(PREFIX)/bin

CC=gcc
CFLAGS=-ansi -Wall -O3 $(INCLUDE) $(LIB)

LINK=-lrokifd `roki-config -l`

COMPILE=$(CC) $(CFLAGS) -o $@ $< $(LINK)

INSTALL=install -m 755

CONFIGGEN=zeda-config-gen
CONFIG=rokifd-config
TARGET=$(CONFIG)

#####################################################################
all : $(TARGET)

$(CONFIG):
	@echo " GENERATE" $@
	-@rm -f $@
	@$(CONFIGGEN) -I $(PREFIX)/include -L $(PREFIX)/lib -l "$(LINK)" -v $(VERSION) > $@

%: %.c
	@echo " CC	" $<
	@$(COMPILE)
install: $(TARGET)
	@echo " INSTALL	" $^
	@$(INSTALL) $^ $(BINDIR)
clean :
	@echo " CLEAN"
	-@rm -f *.o *~ core $(TARGET)
uninstall:
	@echo " UNINSTALL $(TARGET)"
	-@rm -f $(addprefix $(BINDIR)/, $(TARGET))
