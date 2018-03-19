LLVM_CONFIG := llvm-config

THISDIR := $(realpath .)
BUILDDIR := $(THISDIR)/build
INSTALLDIR := $(THISDIR)/install
BINDIR := $(shell $(LLVM_CONFIG) --bindir)

INCLUDE_DIRS := -isystem $(INSTALLDIR)/include

CC := $(BINDIR)/clang
CC_FLAGS := -g -O3 -mthumb -mtune=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard --target=armv7em-none-eabi -MD -MP -ffreestanding -ffunction-sections -nostdlib -nostdlibinc -mimplicit-it=thumb $(INCLUDE_DIRS)

CXX := $(BINDIR)/clang++
CXX_FLAGS := $(CC_FLAGS)

AR := $(BINDIR)/llvm-ar
AR_FLAGS := -rcs

RANLIB := $(BINDIR)/llvm-ranlib

.PHONY: all clean

LIBS := $(INSTALLDIR)/lib/libc.a $(INSTALLDIR)/lib/linux/libclang_rt.builtins-armv7em.a

all: $(LIBS)

$(INSTALLDIR)/lib/libc.a: $(BUILDDIR)/musl/Makefile
	@$(MAKE) AR="$(AR)" ARFLAGS="$(AR_FLAGS)" CC="$(CC)" CFLAGS="$(CC_FLAGS)" RANLIB="$(RANLIB)" -C $(BUILDDIR)/musl all install

$(BUILDDIR)/musl/Makefile:
	@mkdir -p $(@D)
	@cd $(@D) && $(THISDIR)/musl/configure AR="$(AR)" ARFLAGS="$(AR_FLAGS)" CC="$(CC)" CFLAGS="$(CC_FLAGS)" RANLIB="$(RANLIB)" --prefix=$(INSTALLDIR) --target=armv7em-none-eabi --disable-shared

$(INSTALLDIR)/lib/linux/libclang_rt.builtins-armv7em.a: $(BUILDDIR)/compiler-rt/Makefile
	@mkdir -p $(@D)
	@$(MAKE) -C $(BUILDDIR)/compiler-rt all install

$(BUILDDIR)/compiler-rt/Makefile: $(INSTALLDIR)/lib/libc.a
	@mkdir -p $(@D)
	@cd $(@D) && cmake -G "Unix Makefiles" $(THISDIR)/compiler-rt \
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
		-DCMAKE_INSTALL_PREFIX="$(INSTALLDIR)" \
		-DLLVM_CONFIG_PATH="$(LLVM_CONFIG)"

clean:
	rm -rf build install
