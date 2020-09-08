################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
build-1046842913:
	@$(MAKE) --no-print-directory -Onone -f subdir_rules.mk build-1046842913-inproc

build-1046842913-inproc: ../release.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"/Applications/ti/xdctools_3_51_03_28_core/xs" --xdcpath="/Applications/ti/simplelink_cc2640r2_sdk_4_10_00_10/source;/Applications/ti/simplelink_cc2640r2_sdk_4_10_00_10/kernel/tirtos/packages;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC2640R2F -r release -c "/Applications/ti/ccs910/ccs/tools/compiler/ti-cgt-arm_18.12.2.LTS" --compileOptions " -DDeviceFamily_CC26X0R2 " "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-1046842913 ../release.cfg
configPkg/compiler.opt: build-1046842913
configPkg/: build-1046842913


