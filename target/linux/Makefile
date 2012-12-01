export KMODDIR?=       updates
KMODDIR_ARG:=   "INSTALL_MOD_DIR=$(KMODDIR)"
ifneq ($(origin KLIB), undefined)
KMODPATH_ARG:=  "INSTALL_MOD_PATH=$(KLIB)"
else
export KLIB:=          /lib/modules/$(shell uname -r)
endif
export KLIB_BUILD ?=	$(KLIB)/build
export MAKE

DESTDIR?=

ifneq ($(KERNELRELEASE),)

-include $(COMPAT_CONFIG)
include $(COMPAT_CONFIG_CW)

NOSTDINC_FLAGS := -I$(M)/include/ \
	-include $(M)/include/linux/compat-2.6.h \
	$(CFLAGS)

obj-y := compat/


ifeq ($(BT),)


obj-$(CONFIG_COMPAT_NETWORK_MODULES) += src/


ifeq ($(CONFIG_STAGING_EXCLUDE_BUILD),)
endif

endif


else

export PWD :=	$(shell pwd)

# The build will fail if there is any space in PWD.
ifneq (,$(findstring  $() ,$(PWD)))
$(error "The path to this compat-drivers directory has spaces in it." \
	"Please put it somewhere where there is no space")
endif

export CFLAGS += \
	-DCOMPAT_BASE="\"$(shell cat $(PWD)/.compat_base)\"" \
        -DCOMPAT_BASE_TREE="\"$(shell cat $(PWD)/.compat_base_tree)\"" \
        -DCOMPAT_BASE_TREE_VERSION="\"$(shell cat $(PWD)/.compat_base_tree_version)\"" \
        -DCOMPAT_PROJECT="\"Compat-wireless\"" \
        -DCOMPAT_VERSION="\"$(shell cat $(PWD)/.compat_version)\""

# These exported as they are used by the scripts
# to check config and compat autoconf
export COMPAT_CONFIG_CW=$(PWD)/config.mk
export COMPAT_CONFIG=$(PWD)/.config
export CONFIG_CHECK=$(PWD)/.config.mk_md5sum.txt
export COMPAT_AUTOCONF=include/linux/compat_autoconf.h
export CREL=$(shell cat $(PWD)/.compat_version)
export CREL_PRE:=.compat_autoconf_
export CREL_CHECK:=$(PWD)/$(CREL_PRE)$(CREL)

all: linux

$(COMPAT_CONFIG): ;

# Uses compat-drivers to provide backport functionality
# to support the linux-next driver down to all supported
# compat-drivers kernels.
linux: $(CREL_CHECK)
	$(MAKE) -C $(KLIB_BUILD) M=$(PWD) modules
	@touch $@

# We use a CREL_CHECK variable which will depend on the environment used to
# build. If the environment requirements change it forces a reconfiguration
# check.  This means we force a new reconfiguration check if a the user gets a
# new updates of compat-drivers or when the user updates the $(COMPAT_CONFIG)
# file.
# XXX: add kernel target to the CREL_CHECK mix, this would ensure we also
# reconfigure and build again fresh if we detect a new target kernel is
# being used.
$(CREL_CHECK):
	@# Force to regenerate compat autoconf
	+@./compat/scripts/gen-compat-config.sh > $(COMPAT_CONFIG)
	@rm -f $(CONFIG_CHECK)
	+@./scripts/check_config.sh
	@md5sum $(COMPAT_CONFIG_CW) > $(CONFIG_CHECK)
	@touch $@

install: uninstall install-modules install-scripts

install-modules: modules
	$(MAKE) -C $(KLIB_BUILD) M=$(PWD) $(KMODDIR_ARG) $(KMODPATH_ARG) \
		modules_install
	@./scripts/update-initramfs

install-scripts:
	@# All the scripts we can use
	@mkdir -p $(DESTDIR)/usr/lib/compat-drivers/
	@install scripts/modlib.sh	$(DESTDIR)/usr/lib/compat-drivers/
	@# If on distributions like Mandriva which like to
	@# compress their modules this will find out and do
	@# it for you. Reason is some old version of modutils
	@# won't know mac80211.ko should be used instead of
	@# mac80211.ko.gz
	@./scripts/compress_modules
	@# Mandrake doesn't have a depmod.d/ conf file to prefer
	@# the updates/ dir which is what we use so we add one for it
	@# (or any other distribution that doens't have this).
	@./scripts/check_depmod
	@# Udev stuff needed for the new compat_firmware_class.
	@./compat/scripts/compat_firmware_install
	@/sbin/depmod -a
	@echo 
	@echo Now run:
	@echo 
	@echo sudo make unload to unload all: ethernet modules
	@echo
	@echo Run sudo modprobe 'driver-name' to load your desired driver. 
	@echo If unsure reboot.
	@echo

uninstall:
	@# New location, matches upstream
	@rm -rf $(KLIB)/$(KMODDIR)/compat/
	@rm -rf $(KLIB)/$(KMODDIR)/drivers/net/ethernet/atheros/alx/alx.ko*
	@# Lets only remove the stuff we are sure we are providing
	@# on the misc directory.
	@/sbin/depmod -a
	@echo 

clean:
	$(MAKE) -C $(KLIB_BUILD) M=$(PWD) clean
	@rm -f $(CREL_PRE)*
	@rm -rf target/linux/src/
unload:
	@./scripts/unload.sh

.PHONY: all clean install uninstall unload modules Makefile linux linux-src

endif

clean-files += Module.symvers Module.markers modules modules.order
clean-files += $(CREL_CHECK) $(CONFIG_CHECK) $(COMPAT_CONFIG)
