# From bzip2's makefile
BIGFILES=-D_FILE_OFFSET_BITS=64

MYSRCPATHS =
MYINCLUDES = -I. 
MYCFLAGS = -Wall -Winline -O2 -g $(BIGFILES) -DBZ_DEBUG=0
MYDEFS =
MYSRCS = \
	blocksort.c  \
	huffman.c    \
	crctable.c   \
	randtable.c  \
	compress.c   \
	decompress.c \
	bzlib.c

LIB_A = libbz2.a

# Transition: remove old directories and objects
MYCLEANOLDPATH = ../../bzip2

# Resolve "wasm-ld: error: --shared-memory is disallowed by ?.o because it was not compiled with 'atomics' or 'bulk-memory' features."
ifdef EMSCRIPTEN
    MYCFLAGS += -pthread
endif

include ../../../Makefile.host
