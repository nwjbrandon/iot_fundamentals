################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/home/nwjbrandon/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="/home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code" --include_path="/home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code" --include_path="/home/nwjbrandon/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/cc26xxware_2_24_03_17272" --include_path="/home/nwjbrandon/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS/include" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1034585119:
	@$(MAKE) --no-print-directory -Onone -f subdir_rules.mk build-1034585119-inproc

build-1034585119-inproc: ../main.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"/home/nwjbrandon/xdctools_3_32_00_06_core/xs" --xdcpath="/home/nwjbrandon/ti/tirtos_cc13xx_cc26xx_2_21_00_06/packages;/home/nwjbrandon/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/tidrivers_cc13xx_cc26xx_2_21_00_04/packages;/home/nwjbrandon/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/bios_6_46_01_37/packages;/home/nwjbrandon/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/uia_2_01_00_01/packages;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC2650F128 -r release -c "/home/nwjbrandon/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS" --compileOptions "-mv7M3 --code_state=16 --float_support=vfplib -me --include_path=\"/home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code\" --include_path=\"/home/nwjbrandon/Downloads/CS3237_CC2650_skeleton_code/CS3237_CC2650_skeleton_code\" --include_path=\"/home/nwjbrandon/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/cc26xxware_2_24_03_17272\" --include_path=\"/home/nwjbrandon/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS/include\" --define=ccs -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi  " "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-1034585119 ../main.cfg
configPkg/compiler.opt: build-1034585119
configPkg/: build-1034585119


