# Kvaser Linux linlib library Makefile 
# 
#                 Copyright 2012 by Kvaser AB, Mölndal, Sweden
#                         http://www.kvaser.com
# 
#  This software is dual licensed under the following two licenses:
#  BSD-new and GPLv2. You may use either one. See the included
#  COPYING file for details.
# 
#  License: BSD-new
#  ===============================================================================
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#      * Neither the name of the <organization> nor the
#        names of its contributors may be used to endorse or promote products
#        derived from this software without specific prior written permission.
# 
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
#  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# 
#  License: GPLv2
#  ===============================================================================
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# 
#  ---------------------------------------------------------------------------
# 

LIBNAME = liblinlib.so
LIBRARY = $(LIBNAME).1.1.0
SONAME  = $(LIBNAME).1

CFLAGS  += -Wall -DLINLIB_NAME_STRING=\"$(LIBRARY)\" -O2 -fomit-frame-pointer $(XTRA_CFLAGS) -pthread #-DPCMCIA_DEBUG=0
CFLAGS  += -DSPLIT -DLINUX=1 -D_LINUX=1 # maybe remove in time
CFLAGS  += -I. -I../include

# Flags for  reentrant, position independent code
LIBCFLAGS = $(CFLAGS) -D_REENTRANT -fPIC
LDFLAGS = -lc -pthread

#SRCS := $(wildcard *.c)
SRCS := linlib.c 
#SRCS += ../common/osif_functions_user.c

OBJS := $(patsubst %.c, %.o, $(SRCS))
OTHERDEPS := linboot.h
DEPS := $(patsubst %.c, %.d, $(SRCS))


ifeq ($(KV_DEBUG_ON),1)
  KV_XTRA_CFLAGS_DEBUG= -D_DEBUG=1 -DDEBUG=1 
  IS_DEBUG=Debug: $(KV_XTRA_CFLAGS_DEBUG)
  CFLAGS += $(KV_XTRA_CFLAGS_DEBUG)
else
  KV_XTRA_CFLAGS_DEBUG= -D_DEBUG=0 -DDEBUG=0
  CFLAGS += $(KV_XTRA_CFLAGS_DEBUG)
endif


#.PHONY:	all tarball rpm clean examples depend linlib
.PHONY:	all tarball rpm clean depend linlib

#all: $(LIBRARY) examples
all: $(LIBRARY)

linlib: $(LIBRARY)

$(LIBRARY): $(DEPS) $(OTHERDEPS) $(OBJS)
	@echo --------------------------------------------------------------------
	@echo "Building $(LIBRARY) $(IS_DEBUG)"
	$(CC) $(CFLAGS) -shared -Wl,-soname,$(SONAME) -o $@ $(OBJS) $(LDFLAGS)
	ln -sf $(LIBRARY) $(LIBNAME)
	ln -sf $(LIBRARY) $(SONAME)
	@echo --------------------------------------------------------------------

#examples: linlib
#	$(MAKE) -C examples KV_DEBUG_ON=$(KV_DEBUG_ON) sub

%.o: %.c
	$(CC) $(CFLAGS) $(LIBCFLAGS)  -c -o $@ $< 

%.d: %.c
	$(CC) -M $(CFLAGS) $< > $@ 
	$(CC) -M $(CFLAGS) $< | sed s/\\.o/.d/> $@ 

#install: linlib examples
#	$(MAKE) -C examples install
install: linlib
	rm -f /usr/lib/$(LIBNAME)
	rm -f /usr/lib/$(SONAME)
	install  -m 755 $(LIBRARY) /usr/lib/
	ln -sf $(LIBRARY) /usr/lib/$(LIBNAME)
	ln -sf $(LIBRARY) /usr/lib/$(SONAME)
	/sbin/ldconfig
	install -m 644 ../include/linlib.h /usr/include
#	mkdir -p /usr/doc/linlib
#	cp -r ../doc/HTMLhelp /usr/doc/linlib

clean:	
	rm -f $(OBJS) $(LIBRARY) $(DEPS) $(LIBNAME) $(LIBRARY) $(SONAME) $(TARBALL) *~
#	$(MAKE) -C examples clean

depend: $(DEPS)

-include $DEPS	 
