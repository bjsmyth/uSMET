## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,em4f linker.cmd package/cfg/usmet_rtos_pem4f.oem4f

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/usmet_rtos_pem4f.xdl
	$(SED) 's"^\"\(package/cfg/usmet_rtos_pem4fcfg.cmd\)\"$""\"/home/brandon/workspace_v9/uSMET/.config/xconfig_usmet_rtos/\1\""' package/cfg/usmet_rtos_pem4f.xdl > $@
	-$(SETDATE) -r:max package/cfg/usmet_rtos_pem4f.h compiler.opt compiler.opt.defs
