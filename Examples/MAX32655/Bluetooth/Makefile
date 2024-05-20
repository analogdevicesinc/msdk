all:
	@for DIR in ./* ; do $(MAKE) -C $$DIR; done
clean:
	@for DIR in ./* ; do $(MAKE) -C $$DIR distclean; rm -rf $$DIR/build; done
