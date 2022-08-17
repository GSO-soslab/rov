# GPS Time sync with Jetson Xavier NX Develop Kit



## #. Jetson Hardware enable for PPS



### ~. Pre-setup

- Install  [toolchain](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/xavier_toolchain.html) for cross-build kernel

- Download source code

  - Install [SDK manager](https://developer.nvidia.com/nvidia-sdk-manager)

  - Process step 1 and step 2

    ![step1](/home/lin/Develop/doc/notes/step1.png)

    ![step2](/home/lin/Develop/doc/notes/step2.png)

  - After those steps, Jetson system will download and build in **/home/lin/nvidia**

### ~. Configure the kernel

- Setup build environment (we will modify the kernel files and rebuild the Jetson system again)

  ```shell
  cd ~/nvidia
  mkdir kernel_compiled
  # That's the toochain path you installed at Pre-setup
  export CROSS_COMPILE=/[YOUR_INSTALL_PATH]/l4t-gcc/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
  export TEGRA_KERNEL_OUT=/home/[USER]/nvidia/kernel_compiled
  export LOCALVERSION=-tegra
  # Download the source files using "source_sync.sh", will take some time
  cd ~/nvidia/nvidia_sdk/JetPack_4.4.1_Linux_JETSON_XAVIER_NX_DEVKIT/Linux_for_Tegra
  ./source_sync.sh 
  # Use tag tegra-l4t-r32.4.4 because I am using 4.4.1 version
  ```

  

- Enable PPS 

  - Build kernel source configuration 

    ```shell
    cd ~/nvidia/nvidia_sdk/JetPack_4.4.1_Linux_JETSON_XAVIER_NX_DEVKIT/Linux_for_Tegra/sources/kernel/kernel-4.9
    mkdir -p $TEGRA_KERNEL_OUT
    make ARCH=arm64 O=$TEGRA_KERNEL_OUT tegra_defconfig
    ```

  - Add PPS support:

    - Open `/home/lin/nvidia/kernel_compiled/.config`

    - Modify like below:

      ```
      #
      # PPS support
      #
      CONFIG_PPS=y
      CONFIG_PPS_DEBUG=y
      
      #
      # PPS clients support
      #
      CONFIG_PPS_CLIENT_KTIMER=y
      CONFIG_PPS_CLIENT_LDISC=y
      CONFIG_PPS_CLIENT_GPIO=y
      ```

  - Select pin as PPS

    - Open `~/nvidia/nvidia_sdk/JetPack_4.4.1_Linux_JETSON_XAVIER_NX_DEVKIT/Linux_for_Tegra/sources/hardware/nvidia/platform/t19x/jakku/kernel-dts/tegra194-p3668-all-p3509-0000.dts`

    - Modify like below (use **GPIO01**):

      ```
      	pps {
      	    gpios = <&tegra_main_gpio TEGRA194_MAIN_GPIO(Q, 5) GPIO_ACTIVE_LOW>;
      	    compatible = "pps-gpio";
      	    assert-falling-edge; 
      	    status = "okay";
      	};
      ```

- Build the kernel

  ```shell
  cd ~/nvidia/nvidia_sdk/JetPack_4.4.1_Linux_JETSON_XAVIER_NX_DEVKIT/Linux_for_Tegra/sources/kernel/kernel-4.9
  # It will take some time
  make ARCH=arm64 O=$TEGRA_KERNEL_OUT -j8
  ```

  

### ~. Flash the kernel

- Replace the old with new build files

  ```shell
  # replace Image
  cd ~/nvidia/nvidia_sdk/JetPack_4.4.1_Linux_JETSON_XAVIER_NX_DEVKIT/Linux_for_Tegra/kernel/
  cp $TEGRA_KERNEL_OUT/arch/arm64/boot/Image Image
  # replace dtb
  cd dtb
  cp -a /$TEGRA_KERNEL_OUT/arch/arm64/boot/dts/. .
  ```

- Flash

  - Find <target_board> in [Jetson Modules and Configurations](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/quick_start.html#wwpID0EAAPNHA)

  - Find J14 header, connect Pin 9 to Pin 10 to put Kit into **Force Recovery Mode**.

  - connect USB between Jetson and Host computer

  - Power the Jetson, then remove the Pin 9 ~ Pin 10 connection. 

  - Flash to Jetson (directly flash to external device(e.g. SSD) need Jetpack 4.6.1, check [this](https://www.jetsonhacks.com/2021/08/25/native-boot-for-jetson-xaviers/) )

    ```shell
    cd ~/nvidia/nvidia_sdk/JetPack_4.4.1_Linux_JETSON_XAVIER_NX_DEVKIT/Linux_for_Tegra/
    sudo ./flash.sh jetson-xavier-nx-devkit mmcblk0p1
    ```

- Setup 

  - Basic setup the system
  - Install library: `sudo apt-get install nvidia-jetpack`, it will auto install all the lib (e.g. cuda ...).

### ~. Run on SSD



### ~. References

- [GPIO customization on Jetson Platform](https://forums.developer.nvidia.com/t/gpio-customization-on-jetson-platform/163609)
- [Enabling PPS on Jetson Nano](https://msadowski.github.io/pps-support-jetson-nano/)
- [Jetson Kernel Customization](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/kernel_custom.html#)
- [Run from SSD](https://www.jetsonhacks.com/2020/05/29/jetson-xavier-nx-run-from-ssd/)



- [PPS to Xavier NX](https://forums.developer.nvidia.com/t/add-pps-signal-from-gnss-reciever-to-xavier-nx/155347)
- [Time sync Network on Xavier NX](https://forums.developer.nvidia.com/t/time-sensitive-networking-tsn-on-nx/125189/22)



## #.  Software PPS setup



## #. Time sync setup



