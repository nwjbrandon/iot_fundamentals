# invoke SourceDir generated makefile for main.pem3
main.pem3: .libraries,main.pem3
.libraries,main.pem3: package/cfg/main_pem3.xdl
	$(MAKE) -f /home/nwjbrandon/Documents/iot_fundamentals/lab_assignment/code/src/makefile.libs

clean::
	$(MAKE) -f /home/nwjbrandon/Documents/iot_fundamentals/lab_assignment/code/src/makefile.libs clean

