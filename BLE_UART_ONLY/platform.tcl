# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct D:\PSU\Winter_2024\544\fp\ROM_sensor\BLE_UART_ONLY\platform.tcl
# 
# OR launch xsct and run below command.
# source D:\PSU\Winter_2024\544\fp\ROM_sensor\BLE_UART_ONLY\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {BLE_UART_ONLY}\
-hw {D:\PSU\Winter_2024\544\fp\ROM_sensor\vivado\project_2\booleanfpga.xsa}\
-proc {microblaze_0} -os {freertos10_xilinx} -out {D:/PSU/Winter_2024/544/fp/ROM_sensor}

platform write
platform generate -domains 
platform active {BLE_UART_ONLY}
bsp reload
bsp config total_heap_size "8192"
bsp config use_mutexes "true"
bsp write
bsp reload
catch {bsp regenerate}
platform generate
