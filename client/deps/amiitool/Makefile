MYSRCPATHS =
MYINCLUDES = -I. -I.. -I../../../common -I../../../common/mbedtls -I../../../include -I../../src -I../../../include -I../jansson
MYCFLAGS =
MYDEFS =
MYSRCS = \
	amiibo.c \
	drbg.c \
	keygen.c

LIB_A = libamiibo.a

# Transition: remove old directories and objects
MYCLEANOLDPATH = ../../amiitool

# Resolve "wasm-ld: error: --shared-memory is disallowed by ?.o because it was not compiled with 'atomics' or 'bulk-memory' features."
ifdef EMSCRIPTEN
    MYCFLAGS += -pthread
endif

include ../../../Makefile.host

# just for testing amiitool before complete migration into a lib:

amiitool:
	gcc $(CFLAGS) \
	  amiitool.c $(MYSRCS) ../../../common/commonutil.c ../ui.c -lreadline -lm ../../../common/mbedtls/libmbedtls.a \
	  -o amiitool
