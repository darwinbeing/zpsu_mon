# HP HSTNS-PL30 DIY Raspberry Pi Pico Watt Meter

![alt text](https://github.com/darwinbeing/HPServerPSUHack/blob/main/resources/PL30PicoWattMeter.png)


### Build on Mac OS

1. Create an Anaconda virtual environment

2. Installing Dependencies
brew install cmake dtc

2. Install West
pip install west

3. Initialize Zephyr(Tag: v3.4.0)
west init --mr v3.4.0 ~/zephyrproject
cd ~/zephyrproject
west update
pip install -r ~/zephyrproject/zephyr/scripts/requirements.txt

4. Setting Up the Toolchain
https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-darwin-x86_64-arm-none-eabi.tar.xz?rev=a3d8c87bb0af4c40b7d7e0e291f6541b&hash=10927356ACA904E1A0122794E036E8DDE7D8435D

macOS (x86_64) hosted cross toolchains
AArch32 bare-metal target (arm-none-eabi)

arm-gnu-toolchain-13.2.rel1-darwin-x86_64-arm-none-eabi.pkg
arm-gnu-toolchain-13.2.rel1-darwin-x86_64-arm-none-eabi.pkg.asc
arm-gnu-toolchain-13.2.rel1-darwin-x86_64-arm-none-eabi.pkg.sha256asc
arm-gnu-toolchain-13.2.rel1-darwin-x86_64-arm-none-eabi.tar.xz
arm-gnu-toolchain-13.2.rel1-darwin-x86_64-arm-none-eabi.tar.xz.asc
arm-gnu-toolchain-13.2.rel1-darwin-x86_64-arm-none-eabi.tar.xz.sha256asc

export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
export GNUARMEMB_TOOLCHAIN_PATH=~/zephyrproject/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi

5. Clone repo
git clone https://github.com/darwinbeing/zpsu_mon ~/

6. Compile
cd ~/zpsu_mon
source ~/zephyrproject/zephyr/zephyr-env.sh
west build -p -b rpi_pico
