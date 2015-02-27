#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h> // udelay
#include <linux/cdev.h> // cdev
#include <linux/fs.h> // chrdev
#include <linux/uaccess.h>// copy_from/to_user
#include <linux/io.h> // ioremap/iounmap
#include <linux/device.h> // class_create/destroy
#include <linux/moduleparam.h> // module_param
#include "common.h" // common interface with user-side
#include "driver.pio.c" // physical layer
#include "driver.i2c.c" // protocol layer
#include "driver.app.c" // application layer
#include "driver.fop.c" // files operations

dev_t dev;
struct cdev *my_cdev;
struct class *my_class;
struct device *my_device;

int init_module(void){
	int i;
	pio_start();
	
	if (alloc_chrdev_region(&dev,0,COUNT(Devices),MY_NAME"dev")<0)
		return printk(KERN_ALERT "alloc_chrdev_region error\n");//cat /proc/devices
	
	my_cdev = cdev_alloc();
	my_fops=(struct file_operations){.owner = THIS_MODULE,
		.open = my_open,.release = my_close,.unlocked_ioctl = my_ioctl};
	my_cdev->ops = &my_fops;
	cdev_add(my_cdev,dev,COUNT(Devices)); 
	my_class = class_create(THIS_MODULE , MY_NAME"class"); //ls /sys/class
	
	for(i=0;i<COUNT(Devices);i++)//ls /dev/
		device_create(my_class, NULL, MKDEV(MAJOR(dev), MINOR(dev)+i), NULL,
			MY_NAME"%s%i",DevType[Devices[i].type],Devices[i].num);
	
	return 0;
}
void cleanup_module(void){
	int i;
	for(i=0;i<12;i++)
		device_destroy(my_class, MKDEV(MAJOR(dev), MINOR(dev)+i));
	class_destroy(my_class);
	unregister_chrdev_region(dev,COUNT(Devices));
	cdev_del(my_cdev);
	pio_stop();
}

MODULE_LICENSE("GPL");
