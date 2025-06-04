PROJECT ?=DUMMY
DEBUG   ?=n
#arm-linux-gnueabihf-9.1.0-
TOOLCHAIN_VERSION?="9.1.0"
CROSS_COMPILE ?=arm-linux-gnueabihf-sigmastar-9.1.0-
GCC ?= $(CROSS_COMPILE)gcc
G++ ?= $(CROSS_COMPILE)g++
STRIP ?= $(CROSS_COMPILE)strip

DEMO_TOP ?= $(shell pwd)
PROJECT_TOP ?= $(DEMO_TOP)/..

SRC_1_DIR   ?= $(DEMO_TOP)/src/libv4l2
GCC_PARAM    ?= -DMI_DUMMY_TEST
LINUX_TOP    ?=

ifeq ($(DEBUG),y)
GCC_PARAM  += -DUVC_DEBUG_ENABLE
endif

ifeq ($(ALSA_EN),y)
GCC_PARAM  += -DALSA_DEVICE_ENABLE
endif

OBJECT_INCD     ?= $(DEMO_TOP)/inc
OBJECT_SRCD     ?= $(DEMO_TOP)/src
OBJECT_OBJD     ?= $(DEMO_TOP)/obj
OBJECT_BIND     ?= $(DEMO_TOP)/bin

SAMPLE_INCD     ?= $(SRC_1_DIR)/inc
SAMPLE_SRCD     ?= $(SRC_1_DIR)/src
SAMPLE_OBJD     ?= $(SRC_1_DIR)/obj
SAMPLE_LIB      ?= $(SRC_1_DIR)/lib

LINUX_INC       ?= $(LINUX_TOP)/include

EXTERN_INC      ?= $(PROJECT_TOP)/include
EXTERN_LIB      ?= $(PROJECT_TOP)/lib/$(TOOLCHAIN_VERSION)
LIBRARIES       ?= $(DEMO_TOP)/lib
INSTALL_DIR     ?= $(DEMO_TOP)/install

C_INCLUDES +=                  \
    -I$(OBJECT_INCD)           \
    -I$(SAMPLE_INCD)           \
    -I$(LINUX_INC)             \
    -I$(EXTERN_INC)

C_FLAGS := -Wall -O2 -march=armv7-a
C_FLAGS += -fPIC -DPIC -DOMXILCOMPONENTSPATH=\"/$(BUILD_DIR)\" -DCONFIG_DEBUG_LEVEL=255

LDFLAGS := -Wl,--start-group
LDFLAGS += -Wl,-Bdynamic
ifeq ($(ALSA_EN),y)
LDFLAGS += -L$(EXTERN_LIB)/dynamic -lasound
endif
LDFLAGS += -ldl -lpthread -lm -lrt
STLPATH := /opt/arm-linux-gnueabihf-4.8.3-201404/arm-linux-gnueabihf/lib
LDFLAGS += -L$(STLPATH) -lsupc++
LDFLAGS += -Wl,--end-group
#######################################################################################
PROGS = host_uvc

ObjSubst=c
CC=$(GCC)

ifeq ($(ALSA_EN),y)
OBJECT_SRCS :=$(wildcard $(OBJECT_SRCD)/*.$(ObjSubst))
else
OBJECT_SRCS :=$(wildcard $(OBJECT_SRCD)/*[^alsa].$(ObjSubst))
endif
OBJECT_SRCS_NO_DIR=$(notdir $(OBJECT_SRCS))
OBJECTS=$(patsubst %.$(ObjSubst),%.$(ObjSubst).o,$(OBJECT_SRCS_NO_DIR))

SAMPLE_SRCS :=$(wildcard $(SAMPLE_SRCD)/*.$(ObjSubst))
SAMPLE_SRCS_NO_DIR=$(notdir $(SAMPLE_SRCS))
SAMPLE_OBJECTS=$(patsubst %.$(ObjSubst),%.$(ObjSubst).o,$(SAMPLE_SRCS_NO_DIR))

OBJPROG = $(addprefix $(OBJECT_OBJD)/, $(PROGS))

.PHONY: clean prepare PROGS

all: prepare $(OBJPROG)

prepare:
	@echo "OBJECT BIN  :" $(PROGS)
	@echo "OBJECT SRCD :" $(OBJECT_SRCD)
	@echo "OBJECT SRC  :" $(OBJECT_SRCS_NO_DIR)
	@echo "SAMPLE SRCD :" $(SAMPLE_SRCD)
	@echo "SAMPLE SRC  :" $(SAMPLE_SRCS_NO_DIR)

clean:
	@rm -Rf $(OBJECT_OBJD)
	@rm -Rf $(OBJECT_BIND)
	@rm -Rf $(INSTALL_DIR)
	@rm -Rf $(OBJECT_SRCD)/libv4l2/obj
install:
	@mkdir -p $(DEMO_TOP)/install
	@cp -f ${OBJECT_BIND}/* ${INSTALL_DIR}
	@cp -f ${OBJECT_INCD}/*.h  ${INSTALL_DIR}

$(OBJPROG):	$(addprefix $(OBJECT_OBJD)/, $(OBJECTS)) $(addprefix $(SAMPLE_OBJD)/, $(SAMPLE_OBJECTS))
	@mkdir -p $(DEMO_TOP)/bin
	@echo "  BIN $@"
	@$(CC) $(GCC_PARAM) -o $@ $(addprefix $(OBJECT_OBJD)/, $(OBJECTS)) $(addprefix $(SAMPLE_OBJD)/, $(SAMPLE_OBJECTS)) $(LDFLAGS)
	@echo ""
	@mv -f ${OBJECT_OBJD}/$(PROGS) ${OBJECT_BIND}

$(OBJECT_OBJD)/%.$(ObjSubst).o : $(OBJECT_SRCD)/%.$(ObjSubst)
	@mkdir -p obj
	@echo "  CC  $<"
	@$(CC) $(GCC_PARAM) $(C_FLAGS) $(C_INCLUDES) -c $< -o $@

$(SAMPLE_OBJD)/%.$(ObjSubst).o : $(SAMPLE_SRCD)/%.$(ObjSubst)
	@mkdir -p $(SAMPLE_OBJD)
	@echo "  CC  $<"
	@$(CC) $(GCC_PARAM) $(C_FLAGS) $(C_INCLUDES) -c $< -o $@
