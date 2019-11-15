all: projeto-final-cc2650

WITH_UIP6=1
UIP_CONF_IPV6=1
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
CONTIKI = ../../../..
CONTIKI_WITH_IPV6 = 1
CFLAGS += -DUIP_CONF_ND6_SEND_NS=1
CFLAGS += -DUIP_CONF_IPV6_RPL
CFLAGS += -g
DEBUG = 1
APPS += powertrace
PROJECT_SOURCEFILES += mqtt-sn.c
PROJECT_SOURCEFILES += pluv-interruption.c
PROJECT_SOURCEFILES += sensors-helper.c
PROJECT_SOURCEFILES += temp-sensor-helper.c
PROJECT_SOURCEFILES += ../../../../../core/net/llsec/noncoresec/noncoresec.c
include $(CONTIKI)/Makefile.include
