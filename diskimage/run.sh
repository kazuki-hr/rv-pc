#!/bin/sh

PREFIX="/opt/riscv/bin/"

# How to build Berkeley Boot Loader and RISC-V Linux kernel

# (1) File download
wget -nc https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/snapshot/linux-5.0.tar.gz
wget -nc https://github.com/riscv/riscv-pk/archive/v1.0.0.tar.gz

# (2) Unzip the archive files and apply patches
tar -xvf linux-5.0.tar.gz
tar -xvf v1.0.0.tar.gz
patch -d riscv-pk-1.0.0 -p1 < rvpc_patch/riscv-pk.patch

# (3) Copy the kernel config file and build the RISC-V Linux kernel
cd linux-5.0
make mrproper
make ARCH=riscv CROSS_COMPILE=riscv32-unknown-linux-gnu- defconfig
cp ../rvpc_patch/rv32ima_kernel_config .config
make -j$(nproc) V=1 ARCH=riscv CROSS_COMPILE=riscv32-unknown-linux-gnu- vmlinux
cd ../

# (4) build the RISC-V bbl binary
mkdir -p riscv-pk-1.0.0/build
cd riscv-pk-1.0.0/build
../configure -prefix=${PREFIX} --enable-logo --enable-print-device-tree --host=riscv32-unknown-linux-gnu --with-arch=rv32ima --with-payload=../../linux-5.0/vmlinux
make -j$(nproc)
riscv32-unknown-linux-gnu-objcopy -O binary bbl bbl.bin
cd ../../
cp riscv-pk-1.0.0/build/bbl.bin .

# How to build Buildroot disk image

# (1) File download and unzip the downloaded file
wget https://git.busybox.net/buildroot/snapshot/buildroot-2019.11.3.tar.gz
tar -zxvf buildroot-2019.11.3.tar.gz

# (2) Create and fix a configuration file
cd buildroot-2019.11.3
cp ../rvpc_patch/rv32ima_buildroot_config .config
cp -r ../rvpc_patch/overlay/ .

# (3) Build the Buildroot disk image
unset LD_LIBRARY_PATH
make -j$(nproc)
cd ../
cp buildroot-2019.11.3/output/images/rootfs.ext4 root.bin

# How to build Device tree

# (1) File download and unzip the downloaded file
cd devicetree

# (2) Edit device tree source file

# Edit source files if needed

# (3) Build the device tree binary file
dtc -I dts -O dtb devicetree_75mhz.dts -o devicetree_75mhz.dtb
cd ../
cp devicetree/devicetree_75mhz.dtb devicetree.dtb

# How to generate Linux memory initialization file as binary format

# (1) Download the archive file including the C program that creates the memory initialization file
cd initmem_gen
make

# (2) Prepare three required binary files
cp ../bbl.bin .
cp ../root.bin .
cp ../devicetree.dtb .

# (3) Executing a program that creates a memory initialization file
make run
cd ../
cp initmem_gen/initmem.bin .
