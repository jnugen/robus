all:
	cd dynabus ; $(MAKE) all
	cd motor3 ; $(MAKE) all
	cd raspi ; $(MAKE) all

clean:
	cd dynabus ; $(MAKE) clean
	cd motor3 ; $(MAKE) clean
	cd raspi ; $(MAKE) clean
