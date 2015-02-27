volatile Pio_reg_t*pioB;

void pio_wait(void){
	udelay(6);
}
void pio_io(int port,Pio_way_t way){
	pioB->cfg2 &= ~(0b111<<port);//mise a 0 des 3 bits du port
	if(way==OUT)
		pioB->cfg2 |= (1<<port);//remise a 1 si en mode out
	pio_wait();
}
void pio_set(int port,Pio_level_t lv){//SDA_DAT,SCL_DAT
	if(lv==HIGH)pioB->data |= (1<<port);
	if(lv==LOW )pioB->data &=~(1<<port);
	pio_wait();
}
unsigned char pio_get(int port){
	return !!(pioB->data & (1<<port));
}
void pio_ack(Pio_ack_t type){
	pio_io (SDA_CFG,OUT);
	pio_set(SDA_DAT,type);
	pio_set(SCL_DAT,HIGH);
	pio_set(SCL_DAT,LOW);
}
unsigned char pio_recv(Pio_ack_t ack){
	int i;
	unsigned char res = 0;
	pio_io(SDA_CFG,IN);
	for(i=7;i>=0;i--){
		pio_set(SCL_DAT,HIGH);
		res |= pio_get(SDA_DAT) << i;
		pio_set(SCL_DAT,LOW);
	}
	pio_io(SDA_CFG,OUT);
	pio_ack(ack);
	return res;
}
void pio_send(unsigned char data){
	int i;
	for(i=7;i>=0;i--){
		pio_set(SDA_DAT,data & (1<<i) ? HIGH : LOW);
		pio_set(SCL_DAT,HIGH);
		pio_set(SCL_DAT,LOW);
	}
}
void pio_start(void){
	pioB = ioremap(PIO_BASE(PB),sizeof(Pio_reg_t));
	pio_io(SDA_CFG,OUT);
	pio_io(SCL_CFG,OUT);
}
void pio_stop(void){
	iounmap(pioB);
}