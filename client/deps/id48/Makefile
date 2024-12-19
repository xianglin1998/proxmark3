# Makefile for ID48LIB library
MYSRCPATHS =
MYINCLUDES = -I.
MYCFLAGS = -Wpedantic -Wall -Werror -O3 -Wno-unknown-pragmas -Wno-inline -Wno-unused-function
MYDEFS = -DID48_NO_STDIO
MYSRCS = \
	id48_data.c \
	id48_generator.c \
	id48_recover.c

LIB_A = libid48.a

# Resolve "wasm-ld: error: --shared-memory is disallowed by ?.o because it was not compiled with 'atomics' or 'bulk-memory' features."
ifdef EMSCRIPTEN
    MYCFLAGS += -pthread
endif

include ../../../Makefile.host
