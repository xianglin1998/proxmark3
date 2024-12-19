MYSRCPATHS =
MYINCLUDES =
MYCFLAGS =
MYDEFS = -DWAI_PM3_TUNED
MYSRCS = whereami.c

LIB_A = libwhereami.a

# Resolve "wasm-ld: error: --shared-memory is disallowed by ?.o because it was not compiled with 'atomics' or 'bulk-memory' features."
ifdef EMSCRIPTEN
    MYCFLAGS += -pthread
endif

include ../../../Makefile.host
