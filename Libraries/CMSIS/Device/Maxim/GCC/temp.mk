ifeq "$(_OS)" "windows"

else
ifeq "1" "$(shell expr `$(CC) -dumpversion` \>= 12)"
LDFLAGS += -Xlinker --no-warn-rwx-segments
endif
endif
