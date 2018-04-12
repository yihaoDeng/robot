CLEAN_FILES = # deliberately empty, so we can append below.
CXX=g++
LDFLAGS= -lpthread -lrt -lprotobuf
CXXFLAGS=-O0 -g -std=c++11 -fno-builtin-memcmp -msse -msse4.2


DEBUG_LEVEL?=2

ifeq ($(MAKECMDGOALS),dbg)
	DEBUG_LEVEL=2
endif

# compile with -O2 if debug level is not 2
ifneq ($(DEBUG_LEVEL), 2)
OPT += -O2 -fno-omit-frame-pointer
# # if we're compiling for release, compile without debug code (-DNDEBUG) and
# # don't treat warnings as errors
OPT += -DNDEBUG
DISABLE_WARNING_AS_ERROR=1
# # Skip for archs that don't support -momit-leaf-frame-pointer
ifeq (,$(shell $(CXX) -fsyntax-only -momit-leaf-frame-pointer -xc /dev/null 2>&1))
OPT += -momit-leaf-frame-pointer
endif
else
$(warning Warning: Compiling in debug mode. Don't use the resulting binary in production)
OPT += $(PROFILING_FLAGS)
DEBUG_SUFFIX = "_debug"
endif

# ----------------------------------------------
OUTPUT = $(CURDIR)/output
THIRD_PATH = $(CURDIR)/third
SRC_PATH = $(CURDIR)/src

# ----------------Dependences-------------------

ifndef SLASH_PATH
	SLASH_PATH = $(THIRD_PATH)/slash
endif
SLASH = $(SLASH_PATH)/slash/lib/libslash$(DEBUG_SUFFIX).a

SLASH_INCLUDE_DIR=$(SLASH_PATH)
SLASH_LIBRARY=$(SLASH_PATH)/slash/lib/libslash$(DEBUG_SUFFIX).a


ifndef PINK_PATH
	PINK_PATH = $(THIRD_PATH)/pink
endif
PINK = $(PINK_PATH)/pink/lib/libpink$(DEBUG_SUFFIX).a

PINK_INCLUDE_DIR=$(PINK_PATH)
PINK_LIBRARY=$(PINK_PATH)/pink/lib/libpink$(DEBUG_SUFFIX).a

ifndef GLOG_PATH
GLOG_PATH = $(THIRD_PATH)/glog
endif

ifeq ($(360), 1)
	GLOG := $(GLOG_PATH)/.libs/libglog.so.0.0.0
endif

INCLUDE_PATH = -I./include \
			   -I$(SLASH_PATH)/ \
			   -I$(PINK_PATH)/


ifeq ($(360),1)
	INCLUDE_PATH += -I$(GLOG_PATH)/src/
endif

LIB_PATH = -L./ \
		   -L$(SLASH_PATH)/slash/lib/ \
		   -L$(PINK_PATH)/pink/lib/

ifeq ($(360),1)
LIB_PATH += -L$(GLOG_PATH)/.libs/
endif

LDFLAGS += $(LIB_PATH) \
		   -lprotobuf\
		   -lpink$(DEBUG_SUFFIX) \
		   -lslash$(DEBUG_SUFFIX) \
		   -lglog
				
# ---------------End Dependences----------------

#VERSION_CC=$(SRC_PATH)/build_version.cc
LIB_SOURCES := $(wildcard $(SRC_PATH)/*.cc)


#-----------------------------------------------

AM_DEFAULT_VERBOSITY = 0

AM_V_GEN = $(am__v_GEN_$(V))
am__v_GEN_ = $(am__v_GEN_$(AM_DEFAULT_VERBOSITY))
am__v_GEN_0 = @echo "  GEN     " $(notdir $@);
am__v_GEN_1 =
AM_V_at = $(am__v_at_$(V))
am__v_at_ = $(am__v_at_$(AM_DEFAULT_VERBOSITY))
am__v_at_0 = @
am__v_at_1 =

AM_V_CC = $(am__v_CC_$(V))
am__v_CC_ = $(am__v_CC_$(AM_DEFAULT_VERBOSITY))
am__v_CC_0 = @echo "  CC      " $(notdir $@);
am__v_CC_1 =
CCLD = $(CC)
$(warning $(CCLD) $(AM_CFLAGS) $(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS))
LINK = $(CCLD) $(AM_CFLAGS) $(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
AM_V_CCLD = $(am__v_CCLD_$(V))
am__v_CCLD_ = $(am__v_CCLD_$(AM_DEFAULT_VERBOSITY))
am__v_CCLD_0 = @echo "  CCLD    " $(notdir $@);
am__v_CCLD_1 =

AM_LINK = $(AM_V_CCLD)$(CXX) $^ $(EXEC_LDFLAGS) -o $@ $(LDFLAGS)

CXXFLAGS += -g

# This (the first rule) must depend on "all".
default: all

WARNING_FLAGS = -W -Wextra -Wall -Wsign-compare \
				-Wno-unused-parameter -Woverloaded-virtual \
				-Wnon-virtual-dtor -Wno-missing-field-initializers

ifndef DISABLE_WARNING_AS_ERROR
	WARNING_FLAGS += -Werror
endif

CXXFLAGS += $(WARNING_FLAGS) $(INCLUDE_PATH) $(PLATFORM_CXXFLAGS) $(OPT)
$(warning $(CXXFLAGS))

LDFLAGS += $(PLATFORM_LDFLAGS)


LIBOBJECTS = $(LIB_SOURCES:.cc=.o)

# if user didn't config LIBNAME, set the default
ifeq ($(BINNAME),)
	# we should only run pika_hub in production with DEBUG_LEVEL 0
	BINNAME=server$(DEBUG_SUFFIX)
endif
BINARY = ${BINNAME}

.PHONY: distclean clean dbg all


%.o: %.cc
	$(AM_V_CC)$(CXX) $(CXXFLAGS) -c $< -o $@

%.d: %.cc
	@set -e; rm -f $@; $(CXX) -MM $(CXXFLAGS) $< > $@.$$$$; \
		sed 's,\($(notdir $*)\)\.o[ :]*,$(SRC_DIR)/\1.o $@ : ,g' < $@.$$$$ > $@; \
		rm -f $@.$$$$

ifneq ($(MAKECMDGOALS),clean)
  -include $(LIBOBJECTS:.o=.d)
endif

all: $(BINARY)
dbg: $(BINARY)



$(BINARY): $(SLASH) $(PINK) $(GLOG) $(LIBOBJECTS)
	$(AM_V_at)rm -f $@
	$(AM_V_at)$(AM_LINK)
	$(AM_V_at)rm -rf $(OUTPUT)
	$(AM_V_at)mkdir -p $(OUTPUT)/bin
	$(AM_V_at)mv $@ $(OUTPUT)/bin
	$(AM_V_at)cp -r $(CURDIR)/conf $(OUTPUT)

$(PINK):
	$(AM_V_at)make -C $(PINK_PATH)/pink/ DEBUG_LEVEL=$(DEBUG_LEVEL)  SLASH_PATH=$(SLASH_PATH)

$(SLASH):
	$(AM_V_at)make -C $(SLASH_PATH)/slash/ DEBUG_LEVEL=$(DEBUG_LEVEL)

$(GLOG):
	    cd $(THIRD_PATH)/glog; if [ ! -f ./Makefile  ]; then ./configure; fi; make; echo '*' > $(CURDIR)/third/glog/.gitignore;
clean:
	rm -f $(BINARY)
	rm -rf $(CLEAN_FILES)
	find $(SRC_PATH) -name "*.[oda]*" -exec rm -f {} \;
	find $(SRC_PATH) -type f -regex ".*\.\(\(gcda\)\|\(gcno\)\)" -exec rm {} \;

distclean: clean
	make -C $(PINK_PATH)/pink/ clean
	make -C $(SLASH_PATH)/slash/ clean
