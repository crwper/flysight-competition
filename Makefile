#
#             LUFA Library
#     Copyright (C) Dean Camera, 2013.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Define FLYSIGHT_VERSION automatically from source control
VERSION_OPT = -DFLYSIGHT_VERSION='"$(shell git describe --abbrev=6 --dirty --always)"'

# Run "make help" for target help.

MCU          = at90usb646
ARCH         = AVR8
BOARD        = USER
F_CPU        = 8000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = flysight
SRC          = src/Main.c                                                  \
	           src/Config.c                                                \
	           src/Debug.c                                                 \
	           src/Descriptors.c                                           \
	           src/Log.c                                                   \
	           src/Power.c                                                 \
	           src/Signature.c                                             \
	           src/Time.c                                                  \
	           src/Timer.c                                                 \
	           src/uart.c                                                  \
	           src/UBX.c                                                   \
	           src/UsbInterface.c                                          \
	           src/Lib/MMC.c                                               \
	           src/Lib/SCSI.c                                              \
	           vendor/avr-crypto-lib/avr-asm-macros.S                      \
	           vendor/avr-crypto-lib/bigint/bigint-stub.c                  \
	           vendor/avr-crypto-lib/bigint/bigint_asm.S                   \
	           vendor/avr-crypto-lib/bigint/bigint_io.c                    \
	           vendor/avr-crypto-lib/ecdsa/ecc_chudnovsky.c                \
	           vendor/avr-crypto-lib/ecdsa/ecdsa_sign.c                    \
	           vendor/avr-crypto-lib/ecdsa/nist_p192.c                     \
	           vendor/avr-crypto-lib/hfal/hfal-basic.c                     \
	           vendor/avr-crypto-lib/hfal/hfal_sha224.c                    \
	           vendor/avr-crypto-lib/noekeon/noekeon_asm.S                 \
	           vendor/avr-crypto-lib/noekeon/noekeon_prng.c                \
	           vendor/avr-crypto-lib/sha2/sha2_small_common.c              \
	           vendor/avr-crypto-lib/sha2/sha224.c                         \
	           vendor/avr-crypto-lib/test_src/cli-basics.S                 \
	           vendor/avr-crypto-lib/test_src/cli-hexdump.S                \
	           vendor/avr-crypto-lib/test_src/cli-stub.c                   \
	           vendor/avr-crypto-lib/test_src/hexdigit_tab.S               \
	           vendor/FatFS/ff.c                                           \
	           vendor/FatFS/mmc.c                                          \
	           $(LUFA_SRC_USB)                                             \
	           $(LUFA_SRC_USBCLASS) 
LUFA_PATH    = vendor/lufa/LUFA
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -Isrc -Isrc/Config/ -Ivendor -Ivendor/avr-crypto-lib -Ivendor/avr-crypto-lib/bigint -Ivendor/avr-crypto-lib/ecdsa -Ivendor/avr-crypto-lib/hfal -Ivendor/avr-crypto-lib/memxor -Ivendor/avr-crypto-lib/noekeon -Ivendor/avr-crypto-lib/sha2 -Ivendor/avr-crypto-lib/test_src -fdata-sections $(VERSION_OPT)
LD_FLAGS     =

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk

# Target to pack the build results into a single .zip
build.zip: version.txt $(TARGET).bin $(TARGET).eep $(TARGET).elf $(TARGET).hex $(TARGET).lss $(TARGET).map $(TARGET).sym $(TARGET).size.txt
	zip -9q $@ $^

version.txt:
	(git describe --always --dirty; git rev-parse HEAD) > $@

$(TARGET).size.txt: $(TARGET).elf
	$(CROSS)-size --mcu=$(MCU) --format=avr $< > $@
