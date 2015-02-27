BUILD_DIR=/media/storage/ProjetSEPP_CBEL/Kernel/linux-sunxi/TheTree/lib/modules/3.4.103-00033-g9a1cd03/build
ifneq ($(KERNELRELEASE),)
	obj-m := driver_remy.o
else
default:
	$(MAKE) ARCH=arm CROSS_COMPILE=arm-eabi- -C $(BUILD_DIR) M=$(shell pwd) modules
	rm -rf .tmp_versions *.o *.mod.* *.order *.symvers .*.cmd
endif

