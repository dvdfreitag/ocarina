LLVM_CONFIG := llvm-config

THIS_DIR := $(realpath .)
CMSIS_DIR := $(THIS_DIR)/CMSIS
BUILD_DIR := $(THIS_DIR)/build
INSTALL_DIR := $(THIS_DIR)/install
BIN_DIR := $(shell $(LLVM_CONFIG) --bindir)
TEST_DIR := $(THIS_DIR)/test

TEST_SRCS := $(wildcard $(TEST_DIR)/test_*.c)
TEST_OBJS := $(addprefix $(BUILD_DIR)/test/, $(notdir $(TEST_SRCS:.c=.o)))
TEST_BINS := $(basename $(TEST_OBJS))

ifeq ($(shell id -u), 0)
    $(error "This script must not be run as root")
endif

ifeq ($(DEVICE),)
    $(warning "No device set, no device-specific support will be added")
else
    DEVICE_DIR := $(CMSIS_DIR)/devices/$(DEVICE)
    DEVICE_INCLUDE := -I$(DEVICE_DIR)/include
    STARTUP_SRC := $(DEVICE_DIR)/startup_$(DEVICE).c
    STARTUP_OBJ := $(addprefix $(BUILD_DIR)/startup/, $(notdir $(STARTUP_SRC:.c=.o)))
    STARTUP_LIB := $(INSTALL_DIR)/lib/libstartup.a
    LINKER_SCRIPT := -T$(DEVICE_DIR)/$(DEVICE).ld
endif

INCLUDE_DIRS := -isystem $(INSTALL_DIR)/include -isystem $(CMSIS_DIR)/include $(DEVICE_INCLUDE)

CC := $(BIN_DIR)/clang
CC_FLAGS := -g -O3 -mthumb -mtune=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard --target=armv7em-none-eabi -MD -MP -ffreestanding -ffunction-sections -nostdlib -nostdlibinc -mimplicit-it=thumb $(INCLUDE_DIRS)

ifneq ($(DEVICE),)
CC_FLAGS += -D__$(shell echo $(DEVICE) | tr '[:lower:]' '[:upper:]')__
endif

CXX := $(BIN_DIR)/clang++
CXX_FLAGS := $(CC_FLAGS)

AR := $(BIN_DIR)/llvm-ar
AR_FLAGS := rcs

RANLIB := $(BIN_DIR)/llvm-ranlib

LD := ld.lld
LD_FLAGS := --gc-sections --target=armv7em-none-eabi -mcpu=cortex-m7 $(LINKER_SCRIPT)

LIBS := $(INSTALL_DIR)/lib/libc.a $(INSTALL_DIR)/lib/linux/libclang_rt.builtins-armv7em.a $(STARTUP_LIB)

$(BUILD_DIR)/test/test_%.o: $(TEST_DIR)/test_%.c
	@echo [CC] $@
	@$(CC) $(CC_FLAGS) -o $@ -c $<

$(BUILD_DIR)/test/test_%: $(BUILD_DIR)/test/test_%.o
	@echo [LD] $@
	@$(LD) $(LD_FLAGS) -o $@ $< $(addprefix -l, $(LIBS))

.PHONY: all clean tests_dir test_objs tests

all: $(LIBS) 

$(INSTALL_DIR)/lib/libc.a: $(BUILD_DIR)/musl/Makefile
	@$(MAKE) AR="$(AR)" ARFLAGS="$(AR_FLAGS)" CC="$(CC)" CFLAGS="$(CC_FLAGS)" RANLIB="$(RANLIB)" -C $(BUILD_DIR)/musl all install

$(BUILD_DIR)/musl/Makefile:
	@mkdir -p $(@D)
	@cd $(@D) && $(THIS_DIR)/musl/configure AR="$(AR)" ARFLAGS="$(AR_FLAGS)" CC="$(CC)" CFLAGS="$(CC_FLAGS)" RANLIB="$(RANLIB)" --prefix=$(INSTALL_DIR) --target=armv7em-none-eabi --disable-shared

$(INSTALL_DIR)/lib/linux/libclang_rt.builtins-armv7em.a: $(BUILD_DIR)/compiler-rt/Makefile
	@mkdir -p $(@D)
	@$(MAKE) -C $(BUILD_DIR)/compiler-rt all install

$(BUILD_DIR)/compiler-rt/Makefile: $(INSTALL_DIR)/lib/libc.a
	@mkdir -p $(@D)
	@cd $(@D) && cmake -G "Unix Makefiles" $(THIS_DIR)/compiler-rt \
		-DCOMPILER_RT_BAREMETAL_BUILD=ON \
		-DCOMPILER_RT_BUILD_BUILTINS=ON \
		-DCOMPILER_RT_BUILD_SANITIZERS=OFF \
		-DCOMPILER_RT_BUILD_XRAY=OFF \
		-DCOMPILER_RT_BUILD_PROFILE=OFF \
		-DCOMPILER_RT_DEFAULT_TARGET_ONLY=ON \
		-DCMAKE_C_COMPILER="$(CC)" \
		-DCMAKE_C_FLAGS="$(CC_FLAGS)" \
		-DCMAKE_C_COMPILER_TARGET="armv7em-none-eabi" \
		-DCMAKE_CXX_COMPILER="$(CXX)" \
		-DCMAKE_CXX_FLAGS="$(CXX_FLAGS)" \
		-DCMAKE_CXX_COMPILER_TARGET="armv7em-none-eabi" \
		-DCMAKE_ASM_FLAGS="$(CC_FLAGS)" \
		-DCMAKE_AR="$(AR)" \
		-DCMAKE_AR_FLAGS="$(AR_FLAGS)" \
		-DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=lld" \
		-DCMAKE_SYSTEM_NAME=Linux \
		-DCMAKE_INSTALL_PREFIX="$(INSTALL_DIR)" \
		-DLLVM_CONFIG_PATH="$(LLVM_CONFIG)"

ifneq ($(DEVICE),)
$(STARTUP_LIB): $(STARTUP_OBJ)
	@echo [AR] $@
	@$(AR) $(AR_FLAGS) $@ $^

$(STARTUP_OBJ): $(INSTALL_DIR)/lib/libc.a $(INSTALL_DIR)/lib/linux/libclang_rt.builtins-armv7em.a
	@mkdir -p $(@D)
	@echo [CC] $@
	@$(CC) $(CC_FLAGS) -o $@ -c $(STARTUP_SRC)
endif

clean:
	rm -rf build install

test_dir:
	@mkdir -p $(BUILD_DIR)/test

test_objs: test_dir $(TEST_OBJS)

tests: $(LIBS) test_objs $(TEST_BINS)
