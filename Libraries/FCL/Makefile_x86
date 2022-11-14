#
# Makefile for the FCL, Free Cryptographic Library
#

CC=	gcc.exe
CFLAGS= -O2 -W -Wall -DASYMMETRIC -DWORD32 -DX86 -D__x86

IFLAGS= -I ./ -I ./include -I ../include
LFLAGS= -L ./

LIBFLAGS=
OBJ  = main.o ecdsatest.o shatest.o ucl_sha256.o sha256_stone.o ucl_sha384.o ucl_sha512.o sha512_stone.o ucl_sia256.o ucl_sha3.o bignum.o ecdsa_generic_api.o ecdsa_high.o ucl_rng.o ucl_sys.o 
TRACE_OBJ=

#
# Rules
#

all:	fcl_testing.exe

#
# Dependencies
#

%.o:	%.c
	$(CC) $(CFLAGS) $(IFLAGS) -c -o $@ $<

fcl_testing.exe:	$(OBJ)
	$(CC) $(CFLAGS) $(IFLAGS)  $(LFLAGS) -o fcl_testing.exe $(OBJ) $(LIBFLAGS)


# End Of Makefile

