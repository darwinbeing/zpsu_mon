# HP HSTNS-PL30 DIY Raspberry Pi Pico Watt Meter

[//]: # (Image References)
[image1]: ./images/PL30PicoWattMeter.png "PICO Display Pack"
[image2]: ./images/PL30PicoWattMeter2.png "PICO Display Pack2"  
[image3]: ./images/pico_w.png "PICOW WIFI"  

![alt text][image1]
![alt text][image2]

### Build on Mac OS

1. Create an Anaconda virtual environment  
[Download Anaconda for Mac(Intel)](https://repo.anaconda.com/archive/Anaconda3-2024.02-1-MacOSX-x86_64.pkg)  
2. Installing Dependencies  
brew install cmake dtc

2. Install West  
pip install west

3. Initialize Zephyr(Tag: v4.0.0)  
west init --mr v4.0.0 ~/zephyrproject  
cd ~/zephyrproject  
west update  
pip install -r ~/zephyrproject/zephyr/scripts/requirements.txt  

4. Setting Up the Toolchain  
[macOS (x86_64) hosted cross toolchains AArch32 13.2.Rel1](https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-darwin-x86_64-arm-none-eabi.tar.xz?rev=a3d8c87bb0af4c40b7d7e0e291f6541b&hash=10927356ACA904E1A0122794E036E8DDE7D8435D)  
export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb  
export GNUARMEMB_TOOLCHAIN_PATH=~/zephyrproject/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi  

5. Clone repo  
git clone https://github.com/darwinbeing/zpsu_mon.git ~/  

6. Compile  
cd ~/zpsu_mon  
source ~/zephyrproject/zephyr/zephyr-env.sh

    Pico Display Pack  
    west build -b rpi_pico -d build_lcd1 -- -DCONFIG_PICO_DISPLAY_PACK=y
   
    Pico Display Pack2  
    west build -b rpi_pico -d build_lcd2 -- -DCONFIG_PICO_DISPLAY_PACK2=y

### PICO W WIFI


The WiFi functionality on the Pico W is up and running with Zephyr. The chip ID can be read, and the WiFi firmware downloads properly. For faster transmission, the driver needs to support DMA.
![alt text][image3]
