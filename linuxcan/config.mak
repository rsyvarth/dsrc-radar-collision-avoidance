#
# CONFIG MAKEFILE
#

DEPMOD=`which depmod`
UDEVCTRL=`which udevcontrol`
UDEVADM=`which udevadm`

# alter lines below to change default debug level
ifndef KV_VCANOSIF_DEBUG_LEVEL
export KV_VCANOSIF_DEBUG_LEVEL = 8
endif

ifndef KV_PCICAN_DEBUG_LEVEL
export KV_PCICAN_DEBUG_LEVEL    = 8
endif

ifndef KV_USBCAN_DEBUG_LEVEL
export KV_USBCAN_DEBUG_LEVEL    = 8
endif

ifndef KV_PCICAN2_DEBUG_LEVEL
export KV_PCICAN2_DEBUG_LEVEL  = 8
endif

ifndef KV_LEAF_DEBUG_LEVEL
export KV_LEAF_DEBUG_LEVEL      = 8
endif

ifndef KV_MHYDRA_DEBUG_LEVEL
export KV_MHYDRA_DEBUG_LEVEL    = 8
endif

ifndef KV_VIRTUAL_DEBUG_LEVEL
export KV_VIRTUAL_DEBUG_LEVEL   = 8
endif

ifndef KV_PCIEFD_DEBUG_LEVEL
export KV_PCIEFD_DEBUG_LEVEL = 8
endif

#---------------------------------------------------------------------------
# Specific debug flags
KV_PCICAN_ON   += -DPCICAN_DEBUG=$(KV_PCICAN_DEBUG_LEVEL)
KV_USBCAN_ON   += -DUSBCAN_DEBUG=$(KV_USBCAN_DEBUG_LEVEL)
KV_PCICAN2_ON  += -DPCICAN2_DEBUG=$(KV_PCICAN2_DEBUG_LEVEL)
KV_LEAF_ON     += -DLEAF_DEBUG=$(KV_LEAF_DEBUG_LEVEL) -Wno-error=date-time
KV_MHYDRA_ON   += -DMHYDRA_DEBUG=$(KV_MHYDRA_DEBUG_LEVEL)
KV_VIRTUAL_ON  += -DVIRTUAL_DEBUG=$(KV_VIRTUAL_DEBUG_LEVEL)
KV_PCIEFD_ON   += -DPCIEFD_DEBUG=$(KV_PCIEFD_DEBUG_LEVEL)
KV_VCANOSIF_ON += -DVCANOSIF_DEBUG=$(KV_VCANOSIF_DEBUG_LEVEL)

KV_DEBUGFLAGS  = -D_DEBUG=1 -DDEBUG=1 $(KV_PCICAN_ON) $(KV_USBCAN_ON) $(KV_PCICAN2_ON) $(KV_LEAF_ON) $(KV_MHYDRA_ON) $(KV_VIRTUAL_ON) $(KV_PCIEFD_ON) $(KV_VCANOSIF_ON)
KV_NDEBUGFLAGS = -D_DEBUG=0 -DDEBUG=0

#----------------------------------------
# Select kernel source folder
KERNEL_SOURCE_DIR=/lib/modules/`uname -r`/build
KV_KERNEL_SRC_DIR:=$(KERNEL_SOURCE_DIR)

#---------------------------------------------------------------------------
# export these flags to compilation
KV_XTRA_COMMON_FLAGS           = -DLINUX=1 -D_LINUX=1 $(foreach INC,$(INCLUDES),-I$(INC)) -Werror

export KV_XTRA_CFLAGS       = $(KV_XTRA_COMMON_FLAGS) $(KV_NDEBUGFLAGS) -DWIN32=0
export KV_XTRA_CFLAGS_DEBUG = $(KV_XTRA_COMMON_FLAGS) $(KV_DEBUGFLAGS)  -DWIN32=0

# obj files
OBJS := $(patsubst %.c,%.o,$(SRCS))

ifeq ($(KV_DEBUG_ON),1)
  export EXTRA_CFLAGS=$(KV_XTRA_CFLAGS_DEBUG)
  IS_DEBUG=Debug: $(KV_DEBUGFLAGS)
else
  export EXTRA_CFLAGS=$(KV_XTRA_CFLAGS)
endif


#------------------------------------------------------
obj-m := $(KV_MODULE_NAME).o
$(KV_MODULE_NAME)-objs := $(OBJS)

KBUILD_EXTRA_SYMBOLS = $(PWD)/../common/Module.symvers


.PHONY: kv_module install clean

kv_module:
	@echo --------------------------------------------------------------------
	@echo "building $(KV_MODULE_NAME) $(IS_DEBUG)"
	@echo "Kernel src:" $(KV_KERNEL_SRC_DIR)
	$(MAKE) -C $(KV_KERNEL_SRC_DIR) SUBDIRS=$(PWD) modules
	@echo --------------------------------------------------------------------

install: kv_module
	@echo --------------------------------------------------------------------
	@echo "You need to manually install by running 'sudo ./installscript.sh'"
	@echo --------------------------------------------------------------------

clean:
	@echo --------------------------------------------------------------------
	@echo "Cleaning $(KV_MODULE_NAME)" $(IS_DEBUG)
	rm -f $(foreach suffix, o mod.o ko mod.c, $(KV_MODULE_NAME).$(suffix))
	rm -f $(foreach suffix, o.cmd mod.o.cmd ko.cmd, \.$(KV_MODULE_NAME).$(suffix))
	rm -f modules.order Module.symvers new_modules.conf
	rm -rf .tmp_versions
	rm -f $(SRCS:%.c=%.o)
	rm -f $(join $(dir $(SRCS)),$(addprefix \.,$(notdir $(patsubst %.c,%.o.cmd,$(SRCS)))))
	@echo --------------------------------------------------------------------


