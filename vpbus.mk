FPGA_VERSION = 1.0
FPGA_SITE = $(TOPDIR)/package/fpga
FPGA_SITE_METHOD = local
FPGA_LICENSE = GPLv3+

FPGA_DEPENDENCIES = linux

define FPGA_BUILD_CMDS
	$(MAKE) -C $(LINUX_DIR) $(LINUX_MAKE_FLAGS) M=$(@D)
endef

define FPGA_INSTALL_TARGET_CMDS
	$(MAKE) -C $(LINUX_DIR) $(LINUX_MAKE_FLAGS) M=$(@D) modules_install
endef



$(eval $(generic-package))
