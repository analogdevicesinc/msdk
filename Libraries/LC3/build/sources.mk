ifeq "$(LC3_ROOT)" ""
$(error LC3_ROOT must be specified)
endif

VPATH += ${LC3_ROOT}/src

IPATH += ${LC3_ROOT}/include
IPATH += ${LC3_ROOT}/src

SRCS += attdet.c  bits.c  bwdet.c  energy.c  lc3.c  ltpf.c  mdct.c  \
	plc.c  sns.c  spec.c  tables.c  tns.c
