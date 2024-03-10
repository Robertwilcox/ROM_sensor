# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct D:\PSU\Winter_2024\544\fp\ROM_sensor\rom_sensor\platform.tcl
# 
# OR launch xsct and run below command.
# source D:\PSU\Winter_2024\544\fp\ROM_sensor\rom_sensor\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {rom_sensor}\
-hw {D:\PSU\Winter_2024\544\fp\ROM_sensor\vivado\project_2\booleanfpga.xsa}\
-proc {microblaze_0} -os {standalone} -out {D:/PSU/Winter_2024/544/fp/ROM_sensor}

platform write
platform generate -domains 
platform active {rom_sensor}
platform active {rom_sensor}
