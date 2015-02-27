int i2c_wait_ack(void){
	int val;
	pio_set(SDA_DAT,HIGH);
	pio_io(SDA_CFG,IN);
	pio_set(SCL_DAT,HIGH);
	val=pio_get(SDA_DAT);
	pio_set(SCL_DAT,LOW);
	pio_io(SDA_CFG,OUT);
	if(val)printk(KERN_ALERT "ACK ERROR\n");
	return -val;
}
void i2c_sendStart(void){
	pio_set(SDA_DAT,HIGH);
	pio_set(SCL_DAT,HIGH);
	pio_set(SDA_DAT,LOW);
	pio_set(SCL_DAT,LOW);
}
void i2c_sendStop(void){
	pio_set(SDA_DAT,LOW);
	pio_set(SCL_DAT,HIGH);
	pio_set(SDA_DAT,HIGH);
}
void i2c_send(unsigned char data){
	pio_send(data);
	i2c_wait_ack();
}
void i2c_bloc(unsigned char addr,unsigned char cmd,int size,unsigned char*data){
	int i;
	i2c_sendStart();
	i2c_send(addr);
	if((addr&1)==WR)i2c_send(cmd);
	for(i=0;i<size;i++){
		if((addr&1)==RD)data[i]=pio_recv(i<size-1?ACK:NACK);//ACK ACK ACK... NACK
		if((addr&1)==WR)i2c_send(data[i]);
	}
	i2c_sendStop();
}