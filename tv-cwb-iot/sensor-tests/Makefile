DEFINES += PROJECT_CONF_H=\"project-conf.h\"
CFLAGS += -g
CONTIKI_PROJECT = test-sensors
all: $(CONTIKI_PROJECT)

PROJECT_SOURCEFILES += util.c pluv-sensor.c

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
