MAXIM_SBT_DIR=$(shell pwd)/../../Tools/SBT
export MAXIM_SBT_DIR
all:
	@for DIR in ./* ; do $(MAKE) -C $$DIR; cp $$DIR/build/max32570.sbin $$DIR/build/$$DIR.sbin; done
clean:
	@for DIR in ./* ; do $(MAKE) -C $$DIR clean; rm -rf $$DIR/build; done
