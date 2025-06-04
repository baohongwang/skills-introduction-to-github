PROJECT_TOP:=$(shell pwd)
TOOLCHAIN_VERSION="4.8.3"
#CROSS_COMPILE:=arm-linux-gnueabihf-

#export PROJECT_TOP CROSS_COMPILE TOOLCHAIN_VERSION
export PROJECT_TOP
.PHONY: device clean host

OBJECT = host

all: $(OBJECT)
clean: $(foreach obj, $(OBJECT), $(obj)_clean)

$(OBJECT):faker
	$(MAKE) -C $@

$(foreach obj, $(OBJECT), $(obj)_clean):
	@$(MAKE) -C $(subst _clean, ,$@) clean

faker:
