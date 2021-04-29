# default suffices
# .ztk : ZTK format files to describe models
# .zkci: initial state of a kinematic chain
# .zvs : vector sequence
# .zkcs: kinematic chain configuration sequence
# .zef : external force info.

include ../config

INCLUDE=`roki-config -I`
LIB=`roki-config -L`
DEF=`roki-config -D`
BINDIR=$(PREFIX)/bin

CC=gcc
CFLAGS=-ansi -Wall -O3 $(INCLUDE) $(LIB) $(DEF)

LINK=-lroki-fd `roki-config -l`

COMPILE=$(CC) $(CFLAGS) -o $@ $< $(LINK)

INSTALL=install -m 755

CONFIGGEN=zeda-config-gen
CONFIG=roki-fd-config
TARGET=$(CONFIG)

#####################################################################
all : $(TARGET)

$(CONFIG):
	@echo " GENERATE" $@
	-@rm -f $@
	@$(CONFIGGEN) -I "$(INCLUDE)" -L "$(LIB)" -D "$(DEF)" -l "$(LINK)" -v $(VERSION) > $@
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
