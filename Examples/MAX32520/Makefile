all:
	@for DIR in ./* ; do $(MAKE) -C $$DIR; cp $$DIR/build/max32520.sbin $$DIR/build/$$DIR.sbin; done
clean:
	@for DIR in ./* ; do $(MAKE) -C $$DIR distclean; rm -rf $$DIR/build; done
