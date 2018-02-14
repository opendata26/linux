# Wii U Linux
**A port of Linux 4.11.6 to the Wii U baremetal.**

[![pipeline status](https://gitlab.com/linux-wiiu/linux-wiiu/badges/master/pipeline.svg)](https://gitlab.com/linux-wiiu/linux-wiiu/commits/master)

### Getting Linux
##### Prebuilt Download
You can download a prebuilt image [here](https://gitlab.com/linux-wiiu/linux-wiiu/-/jobs/artifacts/master/raw/dtbImage.wiiu?job=linux-build). See [linux-wiiu/linux-loader](https://gitlab.com/linux-wiiu/linux-loader) for your next steps on setting it up.

##### Compiling (Docker)
If you're the type to compile things yourself, you can use our Docker image to sort everything out for you. (Quick warning: the image is ~250MiB on disk)
```sh
#Download linux-wiiu
git clone https://gitlab.com/linux-wiiu/linux-wiiu #or however you want to do that
cd linux-wiiu
#run the docker container
#--rm (clean up after us); -it (use a terminal)
#-v $(pwd):/linux-wiiu (mount the current directory as /linux-wiiu)
docker run --rm -it -v $(pwd):/linux-wiiu quarktheawesome/linux-wiiu-builder
#You should now be in the container - your shell prompt will change
#Configure Linux - the needed make flags are in $LINUXMK to save typing
make wiiu_defconfig $LINUXMK
make -j4 $LINUXMK
#We're done! Exit the container and get back to the host OS
exit
```
Once this completes, you should find dtbImage.wiiu at `arch/powerpc/boot/dtbImage.wiiu`. Check out [linux-wiiu/linux-loader](https://gitlab.com/linux-wiiu/linux-loader) to get this running on your Wii U.

##### Compiling (from scratch - not recommended)
You'll need a PowerPC toolchain. It seems devkitPPC *will not* work. Assuming your chain is `powerpc-linux-gnu-`:
```sh
#clone repo however you'd like
make wiiu_defconfig ARCH=powerpc CROSS_COMPILE=powerpc-linux-gnu- CROSS32_COMPILE=powerpc-linux-gnu-
#if you'd like to make some changes, do menuconfig here
#add -j4 if you'd like
make ARCH=powerpc CROSS_COMPILE=powerpc-linux-gnu- CROSS32_COMPILE=powerpc-linux-gnu-
```
This'll build `arch/powerpc/boot/dtbImage.wiiu`. From here, check on [linux-wiiu/linux-loader](https://gitlab.com/linux-wiiu/linux-loader) for your next steps. There are no kernel modules to worry about, unless you build your own.

### Booting
The kernel commandline is hardcoded (for now) with `root=sda1 rootwait`. This means you'll need to use a USB flash drive as your rootfs. Format it however you'd like (yes, ext4/gpt works) and throw a distro on it - see below for Debian instructions. Plug it and a USB keyboard into the Wii U. Run [linux-wiiu/linux-loader](https://gitlab.com/linux-wiiu/linux-loader) (as described in that repo's README) and enjoy your Linux!

### Device Support
As it stands, we've got:
 - USB OHCI/EHCI (back ports only)
 - Framebuffer graphics (no acceleration or DRI)
 - SD card
 - Bluetooth (tether a phone to get internet)
 - Some ARM interaction (for poweroff/reboot)
 - 2GB of RAM (0x80000000, reclaimed from ARM)
 - One PowerPC core (core 0?)
 - Both interrupt controllers (this is a big deal for us)

Some of our TODOs can be found on [the boards](https://gitlab.com/linux-wiiu/linux-wiiu/boards).

### Distributions and Programs
While we started off developing with Gentoo, we swapped to Debian unstable (everything's precompiled) so that's what we recommend you do too. Debain stable/testing is unbearably outdated on PowerPC so yes, you should use sid/unstable. To make a system, you can use debootstrap or our prebuilt option:

1. Be on Linux. Get a USB (512mb bare minimum) and format it with a single ext4 partition. This will be your rootfs, so make sure it's a decent quality one (speed is important)
2. Download [this archive](https://mega.nz/#!la52GDSS!Y9TnuFmvbWRbFZww7LPvVsyh2egz4CTDyxC2R5r62r4), we'll call it "debian.tar.xz"
3. Mount and cd into your new USB.
4. Run `tar -xvpf <path/to/debian.tar.xz>`. The p is important, you could skip the v.
5. Eject the drive and plug it into your Wii U. With any luck, it'll boot Debian! Log in with username root and password root.

##### Notes
 - This version of Debian is set up to keep the kernel up to date - it'll mount the SD whenever it does this. Remove `deb.heyquark.com` from the apt sources to disable this.
 - X.org works (w/ software rendering) but there's some kind of caching issue that puts lines on moving sections of the screen.
 - If you can come up with a better guide (esp. one including steps for Windows users) feel free to PR it in.
