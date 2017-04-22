VPBUS_VERSION = 0.1
VPBUS_SITE = $(TOPDIR)/package/vpbus
VPBUS_SITE_METHOD = local
VPBUS_LICENSE = GPLv3+

VPBUS_DEPENDENCIES = linux

define VPBUS_BUILD_CMDS
	$(MAKE) -C $(LINUX_DIR) $(LINUX_MAKE_FLAGS) M=$(@D)
endef

define VPBUS_INSTALL_TARGET_CMDS
	$(MAKE) -C $(LINUX_DIR) $(LINUX_MAKE_FLAGS) M=$(@D) modules_install
endef

$(eval $(generic-package))
