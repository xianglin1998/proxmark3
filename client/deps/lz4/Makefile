# From lz4's makefile.
USERCFLAGS:= -O3 $(CFLAGS)
DEBUGFLAGS:= -Wall -Wextra -Wcast-qual -Wcast-align -Wshadow \
             -Wswitch-enum -Wdeclaration-after-statement -Wstrict-prototypes \
             -Wundef -Wpointer-arith -Wstrict-aliasing=1

MYSRCPATHS =
MYINCLUDES = -I. 
MYCFLAGS = $(DEBUGFLAGS) $(USERCFLAGS)
MYDEFS =
MYSRCS = \
	lz4.c \
	lz4file.c \
	lz4frame.c \
	lz4hc.c \
	xxhash.c \

LIB_A = liblz4.a

# Transition: remove old directories and objects
MYCLEANOLDPATH = ../../lz4

# Resolve "wasm-ld: error: --shared-memory is disallowed by ?.o because it was not compiled with 'atomics' or 'bulk-memory' features."
ifdef EMSCRIPTEN
    MYCFLAGS += -pthread
endif

include ../../../Makefile.host
