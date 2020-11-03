#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#
#  target compatibility key = ti.targets.arm.elf.M3{1,0,20.2,1
#
ifeq (,$(MK_NOGENDEPS))
-include package/cfg/main_pem3.oem3.dep
package/cfg/main_pem3.oem3.dep: ;
endif

package/cfg/main_pem3.oem3: | .interfaces
package/cfg/main_pem3.oem3: package/cfg/main_pem3.c package/cfg/main_pem3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clem3 $< ...
	$(ti.targets.arm.elf.M3.rootDir)/bin/armcl -c  -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="/home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code" --include_path="/home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code" --include_path="/home/nwjbrandon/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/cc26xxware_2_24_03_17272" --include_path="/home/nwjbrandon/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS/include" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi   -qq -pdsw225 -ms --fp_mode=strict --endian=little -mv7M3 --abi=eabi -eo.oem3 -ea.sem3   -Dxdc_cfg__xheader__='"configPkg/package/cfg/main_pem3.h"'  -Dxdc_target_name__=M3 -Dxdc_target_types__=ti/targets/arm/elf/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_20_2_1 -O2  $(XDCINCS) -I$(ti.targets.arm.elf.M3.rootDir)/include  -fs=./package/cfg -fr=./package/cfg -fc $<
	$(MKDEP) -a $@.dep -p package/cfg -s oem3 $< -C   -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="/home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code" --include_path="/home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code" --include_path="/home/nwjbrandon/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/cc26xxware_2_24_03_17272" --include_path="/home/nwjbrandon/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS/include" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi   -qq -pdsw225 -ms --fp_mode=strict --endian=little -mv7M3 --abi=eabi -eo.oem3 -ea.sem3   -Dxdc_cfg__xheader__='"configPkg/package/cfg/main_pem3.h"'  -Dxdc_target_name__=M3 -Dxdc_target_types__=ti/targets/arm/elf/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_20_2_1 -O2  $(XDCINCS) -I$(ti.targets.arm.elf.M3.rootDir)/include  -fs=./package/cfg -fr=./package/cfg
	-@$(FIXDEP) $@.dep $@.dep
	
package/cfg/main_pem3.oem3: export C_DIR=
package/cfg/main_pem3.oem3: PATH:=$(ti.targets.arm.elf.M3.rootDir)/bin/:$(PATH)

package/cfg/main_pem3.sem3: | .interfaces
package/cfg/main_pem3.sem3: package/cfg/main_pem3.c package/cfg/main_pem3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clem3 -n $< ...
	$(ti.targets.arm.elf.M3.rootDir)/bin/armcl -c -n -s --symdebug:none -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="/home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code" --include_path="/home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code" --include_path="/home/nwjbrandon/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/cc26xxware_2_24_03_17272" --include_path="/home/nwjbrandon/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS/include" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi   -qq -pdsw225 --endian=little -mv7M3 --abi=eabi -eo.oem3 -ea.sem3   -Dxdc_cfg__xheader__='"configPkg/package/cfg/main_pem3.h"'  -Dxdc_target_name__=M3 -Dxdc_target_types__=ti/targets/arm/elf/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_20_2_1 -O2  $(XDCINCS) -I$(ti.targets.arm.elf.M3.rootDir)/include  -fs=./package/cfg -fr=./package/cfg -fc $<
	$(MKDEP) -a $@.dep -p package/cfg -s oem3 $< -C  -n -s --symdebug:none -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="/home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code" --include_path="/home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code" --include_path="/home/nwjbrandon/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/cc26xxware_2_24_03_17272" --include_path="/home/nwjbrandon/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS/include" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi   -qq -pdsw225 --endian=little -mv7M3 --abi=eabi -eo.oem3 -ea.sem3   -Dxdc_cfg__xheader__='"configPkg/package/cfg/main_pem3.h"'  -Dxdc_target_name__=M3 -Dxdc_target_types__=ti/targets/arm/elf/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_20_2_1 -O2  $(XDCINCS) -I$(ti.targets.arm.elf.M3.rootDir)/include  -fs=./package/cfg -fr=./package/cfg
	-@$(FIXDEP) $@.dep $@.dep
	
package/cfg/main_pem3.sem3: export C_DIR=
package/cfg/main_pem3.sem3: PATH:=$(ti.targets.arm.elf.M3.rootDir)/bin/:$(PATH)

clean,em3 ::
	-$(RM) package/cfg/main_pem3.oem3
	-$(RM) package/cfg/main_pem3.sem3

main.pem3: package/cfg/main_pem3.oem3 package/cfg/main_pem3.mak

clean::
	-$(RM) package/cfg/main_pem3.mak
