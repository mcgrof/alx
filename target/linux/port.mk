TARGET_LINUX=target/linux/

all: help

linux-help:
	@echo "    make linux-src              - Transforms code for integration into linux-next"
	@echo "    make linux                  - Builds alx for any Linux kernel 2.6.28 - 3.x"
	@echo "    make linux-install          - Install linux target"

# Convert unified driver code to Linux, always targeting linux-next
linux-src:
	@cp -a src $(TARGET_LINUX)

linux-backport:
	@-cd $(TARGET_LINUX) ; \
	  ./refresh-compat ; \
	  patch -N -p6 -d src/ < patches/unified-drivers/network/0001-backport-alx.patch

# Uses compat-drivers to provide backport functionality
# to support the linux-next driver down to all supported
# compat-drivers kernels.
linux: linux-backport
	make -C $(TARGET_LINUX)

install-linux:
	$(MAKE) -C $(TARGET_LINUX) install-modules

uninstall-linux:
	$(MAKE) -C $(TARGET_LINUX) uninstall

.PHONY: linux-help linux-src linux-backport linux install-linux uninstall-linux
