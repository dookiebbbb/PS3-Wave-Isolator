#---------------------------------------------------------------------------------
# PS3 Wave Screensaver — Makefile
# Compatible with ps3dev/ps3dev Docker image and local ps3toolchain installs
#---------------------------------------------------------------------------------

TARGET      := ps3wave
TITLE       := PS3 Wave Screensaver
APPID       := WAVE00001
CONTENTID   := UP0001-WAVE00001_00-0000000000000000
VERSION     := 01.00

BUILD       := build
SOURCE      := source
PKG_DIR     := pkg

#---------------------------------------------------------------------------------
# Toolchain paths (auto-detected from environment, or set defaults)
#---------------------------------------------------------------------------------

PS3DEV      ?= /usr/local/ps3dev
PSL1GHT     ?= $(PS3DEV)/ppu
PORTLIBS    ?= $(PS3DEV)/portlibs/ppu

CC          := $(PSL1GHT)/bin/ppu-gcc
STRIP       := $(PSL1GHT)/bin/ppu-strip
FSELF       := python3 $(PS3DEV)/bin/fself.py
PKG_TOOL    := python3 $(PS3DEV)/bin/pkg.py
SFO_TOOL    := python3 $(PS3DEV)/bin/sfo.py

#---------------------------------------------------------------------------------
# Compiler and linker flags
#---------------------------------------------------------------------------------

CFLAGS  := -O2 -Wall -mcpu=cell \
            -I$(PSL1GHT)/include \
            -I$(PORTLIBS)/include \
            -DGCM_LABEL_INDEX=255

LDFLAGS := -L$(PSL1GHT)/lib \
            -L$(PORTLIBS)/lib \
            -lrsx -lgcm_sys -lsysutil -lpad \
            -lio -lsys -lm -lz -lrt -lc

SRCS    := $(wildcard $(SOURCE)/*.c)
OBJS    := $(SRCS:$(SOURCE)/%.c=$(BUILD)/%.o)

#---------------------------------------------------------------------------------

.PHONY: all clean pkg

all: $(TARGET).self

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/%.o: $(SOURCE)/%.c | $(BUILD)
	@echo "  CC   $<"
	@$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJS)
	@echo "  LD   $@"
	@$(CC) $(OBJS) $(LDFLAGS) -o $@

$(TARGET).self: $(TARGET).elf
	@echo "  STRIP  $(TARGET).elf"
	@$(STRIP) $(TARGET).elf -o $(TARGET)_stripped.elf
	@echo "  FSELF  $(TARGET).self"
	@$(FSELF) $(TARGET)_stripped.elf $(TARGET).self
	@echo "  Built: $(TARGET).self"

pkg: $(TARGET).self
	@echo "  Building PKG..."
	@mkdir -p $(PKG_DIR)/USRDIR
	@cp $(TARGET).self $(PKG_DIR)/USRDIR/EBOOT.BIN
	@echo "  Generating PARAM.SFO..."
	@$(SFO_TOOL) \
		--title    "$(TITLE)" \
		--appid    "$(APPID)" \
		--version  "$(VERSION)" \
		$(PKG_DIR)/PARAM.SFO
	@echo "  Packaging..."
	@$(PKG_TOOL) \
		--contentid $(CONTENTID) \
		$(PKG_DIR)/ \
		$(TARGET).pkg
	@echo "  Done: $(TARGET).pkg"

clean:
	@rm -rf $(BUILD) \
	        $(TARGET).elf \
	        $(TARGET)_stripped.elf \
	        $(TARGET).self \
	        $(TARGET).pkg \
	        $(PKG_DIR)/USRDIR/EBOOT.BIN \
	        $(PKG_DIR)/PARAM.SFO
	@echo "  Cleaned"
