DEFINES += PROJECT_CONF_H=\"project-conf.h\"
CFLAGS += -g
CONTIKI_PROJECT = test-sensors
all: $(CONTIKI_PROJECT)

PROJECT_SOURCEFILES += interruption-sensor.c
PROJECT_SOURCEFILES += temp-sensor-helper.c
PROJECT_SOURCEFILES += sensors-helper.c
PROJECT_SOURCEFILES += ../../../core/net/llsec/noncoresec/noncoresec.c

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
