DIRNAME    = $(shell basename $(shell pwd))
PACKAGE    = $(firstword $(subst -, , $(DIRNAME)))
LIBS       =
VERSION    = v1.0
ADD_CFLAGS =
ADD_LFLAGS =
EXTRA_DIST =

PREFIX     := arm-none-eabi-
CC         := $(PREFIX)gcc
LD         := $(PREFIX)ld
CXX        := $(PREFIX)g++
PKG        := $(PREFIX)pkgconfig
OBJCOPY    := $(PREFIX)objcopy

ifneq ($(strip $(LIBS)),)
PKG_CFLAGS = $(shell $(PKG) $(LIBS) --cflags)
PKG_LFLAGS = $(shell $(PKG) $(LIBS) --libs)
else
PKG_CFLAGS =
PKG_LFLAGS =
endif

DESTDIR ?= /usr
PREFIX ?= $(DESTDIR)

LDSCRIPT = KL02Z32-bl.ld
DBG_CFLAGS = -ggdb -g -DDEBUG -Wall
DBG_LFLAGS = -ggdb -g -Wall
CFLAGS     = $(ADD_CFLAGS) $(PKG_CFLAGS) \
             -DVERSION=\"$(VERSION)\" -DPACKAGE=\"$(PACKAGE)\" \
             -DPREFIX=\"$(PREFIX)\" -DDESTDIR=\"$(DESTDIR)\" -DARDUINO=160 \
             -fsingle-precision-constant -I. \
             -mcpu=cortex-m0plus -mfloat-abi=soft -mthumb \
             -DARDUINO_APP -fno-builtin \
             -ffunction-sections -fdata-sections -fno-common \
             -fomit-frame-pointer -falign-functions=16 -nostdlib -Os
CXXFLAGS   = $(CFLAGS) -std=c++11 -fno-rtti -fno-exceptions
LFLAGS     = $(ADD_LFLAGS) $(PKG_LFLAGS) $(CFLAGS) \
             -nostartfiles -nostdlib -nodefaultlibs \
             -Wl,-Map=$(PROJECT).map,--gc-sections \
             -Wl,--cref,--no-warn-mismatch,--script=$(LDSCRIPT),--build-id=none

OBJ_DIR    = .obj/
DIST_FILES = Makefile .version version.sh install.sh \
             README.md AUTHORS COPYING ChangeLog.txt \
             $(wildcard *.c) $(wildcard *.h) $(EXTRA_DIST)

CSOURCES   = $(wildcard *.c)
CPPSOURCES = $(wildcard *.cpp)
ASOURCES   = $(wildcard *.s)
COBJS      = $(addprefix $(OBJ_DIR)/, $(notdir $(CSOURCES:.c=.o)))
CXXOBJS    = $(addprefix $(OBJ_DIR)/, $(notdir $(CPPSOURCES:.cpp=.o)))
AOBJS      = $(addprefix $(OBJ_DIR)/, $(notdir $(ASOURCES:.s=.o)))
OBJECTS    = $(COBJS) $(CXXOBJS) $(AOBJS)
VPATH = .

QUIET      = @

ALL        = all
TARGET     = $(PACKAGE).elf
DEBUG      = debug
REBUILD    = rebuild
DREBUILD   = drebuild
CLEAN      = clean
CHANGELOG  = ChangeLog.txt
DISTCLEAN  = distclean
DIST       = dist
DDIST      = dailydist
INSTALL    = install
INIT       = init

$(ALL): $(TARGET)

$(OBJECTS): | $(OBJ_DIR)

$(TARGET): $(OBJECTS) $(LDSCRIPT)
	$(QUIET) echo "  LD        $@"
	$(QUIET) $(CXX) $(OBJECTS) $(LFLAGS) -o $@
	$(QUIET) echo "  OBJCOPY   $(PACKAGE).bin"
	$(QUIET) $(OBJCOPY) -O binary $@ $(PACKAGE).bin
	$(QUIET) echo "  SYMBOL    $(PACKAGE).symbol"
	$(QUIET) $(OBJCOPY) --only-keep-debug $< $(PACKAGE).symbol 2> /dev/null



$(DEBUG): CFLAGS += $(DBG_CFLAGS)
$(DEBUG): LFLAGS += $(DBG_LFLAGS)
CFLAGS += $(DBG_CFLAGS)
LFLAGS += $(DBG_LFLAGS)
$(DEBUG): $(TARGET)

$(OBJ_DIR):
	$(QUIET) mkdir -p $(OBJ_DIR)

$(COBJS) : $(OBJ_DIR)/%.o : %.c Makefile
	$(QUIET) echo "  CC        $<	$(notdir $@)"
	$(QUIET) $(CC) -c $< $(CFLAGS) -o $@ -MMD

$(OBJ_DIR)/%.o: %.cpp
	$(QUIET) echo "  CXX       $<	$(notdir $@)"
	$(QUIET) $(CXX) -c $< $(CXXFLAGS) -o $@ -MMD

$(OBJ_DIR)/%.o: %.s
	$(QUIET) echo "  AS        $<	$(notdir $@)"
	$(QUIET) $(CC) -x assembler-with-cpp -c $< $(CFLAGS) -o $@ -MMD

.PHONY: $(CLEAN) $(DISTCLEAN) $(DIST) $(REBUILD) $(DREBUILD) $(INSTALL) \
        $(CHANGELOG) $(INIT)

$(CLEAN):
	$(QUIET) rm -f $(wildcard $(OBJ_DIR)/*.d)
	$(QUIET) rm -f $(wildcard $(OBJ_DIR)/*.o)
	$(QUIET) rm -f $(TARGET) $(PACKAGE).bin $(PACKAGE).symbol

$(DISTCLEAN): $(CLEAN)
	$(QUIET) rm -rf $(OBJ_DIR) $(wildcard $(TARGET)-*.tar.gz)
	$(QUIET) rm -f $(CHANGELOG)

$(REBUILD):  $(CLEAN) $(TARGET)
$(DREBUILD): $(CLEAN) $(DEBUG)

$(CHANGELOG):
	$(QUIET) if [ -d .git ] ; then \
		git log `git tag`.. --pretty=format:"* %ad | %s%d [%an]" \
		--date=short > $@ ; \
	fi
	$(QUIET) echo "" >> $@

$(DIST): DIST_NAME = $(TARGET)-$(VERSION).tar.gz
$(DIST): CURR_PWD  = $(shell pwd)
$(DIST): $(CHANGELOG)
$(DIST):
	$(QUIET) echo "Making $(DIST_NAME)"
	$(QUIET) mkdir $(CURR_PWD)/dist/$(TARGET) -p
	$(QUIET) cp $(DIST_FILES) $(CURR_PWD)/dist/$(TARGET) -f
	$(QUIET) cd $(CURR_PWD)/dist/ && \
                 tar -czvf $(DIST_NAME) $(TARGET) > /dev/null
	$(QUIET) mv $(CURR_PWD)/dist/$(DIST_NAME) $(CURR_PWD) && \
                 rm -rf dist

$(DDIST): DIST_NAME = $(TARGET)-$(VERSION)-$(shell date +%d%m%y).tar.gz
$(DDIST): CURR_PWD  = $(shell pwd)
$(DDIST): $(CHANGELOG)
$(DDIST):
	$(QUIET) echo "Making $(DIST_NAME)"
	$(QUIET) mkdir $(CURR_PWD)/dist/$(TARGET) -p
	$(QUIET) cp $(DIST_FILES) $(CURR_PWD)/dist/$(TARGET) -f
	$(QUIET) cd $(CURR_PWD)/dist/ && \
                 tar -czvf $(DIST_NAME) $(TARGET) > /dev/null
	$(QUIET) mv $(CURR_PWD)/dist/$(DIST_NAME) $(CURR_PWD) && \
                 rm -rf dist

$(INSTALL):
	$(QUIET) echo "Installing $(TARGET)..."
	$(QUIET) ./install.sh --target "$(TARGET)" --version "$(VERSION)" --prefix "$(PREFIX)" --dest-dir "$(DESTDIR)"

$(UNINSTALL):
	$(QUIET) echo "Uninstalling $(TARGET)..."
	$(QUIET) ./uninstall.sh --target "$(TARGET)" --version "$(VERSION)" --prefix "$(PREFIX)" --dest-dir "$(DESTDIR)"

# #RAI - Remove After Init, will be automatically removed after 'make init' executing
$(INIT):                                                               #RAI
	$(QUIET) rm -rf .git                                           #RAI
	$(QUIET) git init                                              #RAI
	$(QUIET) echo "" > AUTHORS                                     #RAI
	$(QUIET) echo "" > README.md                                   #RAI
	$(QUIET) echo "\n\n# Project-specific" >> .gitignore           #RAI
	$(QUIET) echo "$(TARGET)" >> .gitignore                        #RAI
	$(QUIET) sed -i '/#RAI/d' Makefile                             #RAI
	$(QUIET) git add -A && git commit -am 'initial commit(empty)'  #RAI
	$(QUIET) git tag v0.0                                          #RAI

include $(wildcard $(OBJ_DIR)/*.d)
