TARGET_LINUX=target/linux/

all: help

linux-help:
	@echo "======================================================================="
	@echo "Linux build instructions"
	@echo ""
	@echo "If you want to build and install alx you should issue these commands:"
	@echo ""
	@echo "    make linux"
	@echo "    make install-linux"
	@echo ""
	@echo "The full list of Linux targets availale:"
	@echo ""
	@echo "    make linux-src              - Transforms code for integration into linux-next"
	@echo "    make linux                  - Builds alx for any Linux kernel 2.6.28 - 3.x"
	@echo "    make install-linux          - Install linux target"
	@echo "    make uninstall-linux        - Install linux target"
	@echo "    make clean-linux            - Install linux target"
	@echo "======================================================================="

# Convert unified driver code to Linux, always targeting linux-next
linux-src:
	@cp -a src $(TARGET_LINUX)

linux-backport: linux-src
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

clean-linux:
	$(MAKE) -C $(TARGET_LINUX) clean

uninstall-linux:
	$(MAKE) -C $(TARGET_LINUX) uninstall

.PHONY: linux-help linux-src linux-backport linux install-linux uninstall-linux
