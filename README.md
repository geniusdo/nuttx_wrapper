# Usage
Open menuconfig: `cmake --build . --target menuconfig`.  
Save defconfig: `cmake --build . --target saveconfig`.  
Download: `dfu-util -a 0 -s 0x08000000:leave -D prototype.bin`  
Compile twice surprise.
