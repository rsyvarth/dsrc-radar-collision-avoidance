#
# Makefile virtualcan
#

-include $(PWD)/../kveyes.mak

# module name
KV_MODULE_NAME = kvvirtualcan

# Includes
INCLUDES := $(src)/../include/

# source files
SRCS :=           virtualcan.c 

ifneq ($(KERNELRELEASE),)
	RUNDIR := $(src)
else
	RUNDIR := $(PWD)
endif

include $(RUNDIR)/../config.mak
