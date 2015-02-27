struct file_operations my_fops;

ssize_t my_read(struct file *f, char *buf, size_t size, loff_t *offset){
	int i,minor=iminor(f->f_path.dentry->d_inode);
	unsigned char val=0,num=Devices[minor].num;
	
	printk(KERN_ALERT "%s(%s%i)\n",__FUNCTION__,NAME(minor));
	for(i=0;i<size;i++){
		switch(Devices[minor].type){
			case REL:val= read_rel(num);break;
			case OPT:val= read_opt(num);break;
			case ANL:val= read_anl(num);break;
			default:break;
		}
		if(copy_to_user(buf+i, &val, sizeof(val)))break;
	}
	return i;
}
ssize_t my_write(struct file *f, const char *buf, size_t size, loff_t *offset){
	int i,minor=iminor(f->f_path.dentry->d_inode);
	char val=0;
	
	for(i=0;i<size;i++){
		if(copy_from_user(&val, buf+i, sizeof(val)))break;
		printk(KERN_ALERT "%s(%s%i,%i)\n",__FUNCTION__, NAME(minor),val);
		switch(Devices[minor].type){
			case REL:write_rel(Devices[minor].num,val?HIGH:LOW);break;
			default:break;
		}
	}
	return i;
}
long my_ioctl(struct file *f, unsigned int cmd, unsigned long arg){
	int minor=iminor(f->f_path.dentry->d_inode);
	printk(KERN_ALERT "%s(%s%i,%i,%lu)\n",__FUNCTION__, NAME(minor),cmd,arg);
	switch(cmd){
		case SET_ADDR:set_addr(arg);break;
		default:break;
	}
	return 0;
}
int my_open(struct inode *in, struct file *f){
	void*all_read []={my_read,my_read,my_read};
	void*all_write[]={my_write,NULL,NULL};
	
	if(iminor(in)>COUNT(Devices))return -1;
	my_fops.read  = all_read [Devices[iminor(in)].type];
	my_fops.write = all_write[Devices[iminor(in)].type];
	printk(KERN_ALERT "%s(%s%i) [%c%c%c]\n",__FUNCTION__,NAME(iminor(in)),
		my_fops.read?'r':'-',my_fops.write?'w':'-',my_fops.unlocked_ioctl?'x':'-');
	return 0;
}
int my_close(struct inode *in, struct file *f){
	printk(KERN_ALERT "%s(%s%i)\n",__FUNCTION__,NAME(iminor(in)));
	return 0;
}