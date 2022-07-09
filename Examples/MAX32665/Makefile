all:
	@for DIR in ./BLE* ; do $(MAKE) -C $$DIR; done
clean:
	@for DIR in ./BLE* ; do $(MAKE) -C $$DIR clean; rm -rf $$DIR/build; done
