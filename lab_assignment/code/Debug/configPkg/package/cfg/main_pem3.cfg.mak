# invoke SourceDir generated makefile for main.pem3
main.pem3: .libraries,main.pem3
.libraries,main.pem3: package/cfg/main_pem3.xdl
	$(MAKE) -f /home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code/src/makefile.libs

clean::
	$(MAKE) -f /home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code/src/makefile.libs clean

