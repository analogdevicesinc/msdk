MAXIM_SBT_DIR=$(shell pwd)/../../Tools/SBT
export MAXIM_SBT_DIR
all:
	@for DIR in ./* ; do $(MAKE) -C $$DIR; done
clean:
	@for DIR in ./* ; do $(MAKE) -C $$DIR distclean; rm -rf $$DIR/build; done
