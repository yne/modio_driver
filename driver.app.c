#define SLAVE_ADDR(a,i) ((a<<1)|i)
static int p_addr=MODIO_ID;
module_param(p_addr, int, S_IRUGO);

unsigned char read_opt(int num){
	unsigned char states;
	i2c_bloc(SLAVE_ADDR(p_addr,WR),MODIO_ADDR_OPT,0,NULL);
	i2c_bloc(SLAVE_ADDR(p_addr,RD),MODIO_ADDR_OPT,sizeof(states),&states);
	return !!(states&(1<<num));
}
unsigned char read_anl(int num){
	unsigned char i,res=0,val[2];//low,high
	i2c_bloc(SLAVE_ADDR(p_addr,WR),MODIO_ADDR_ANL+num,0,NULL);
	i2c_bloc(SLAVE_ADDR(p_addr,RD),MODIO_ADDR_ANL+num,sizeof(val),val);
	//inverse les bits 0 a 9
	for(i=0;i<8;i++){//sur les 8 premiers
		res |= (!!(val[0] & 0x80)) << i;
		val[0] <<= 1;
	}
	//et sur les 2 derniers
	res |= (!!(val[1] & 0b10)) << 8;
	res |= (!!(val[1] & 0b01)) << 9;

	return res>>2;
}
unsigned char rel_state=0;//so we know each relay states
unsigned char read_rel(unsigned char num){
	return (rel_state >>num)&1;
}
void write_rel(unsigned char num,Pio_level_t lv){
	if(lv==HIGH)rel_state |=  (1<<num);
	if(lv==LOW )rel_state &= ~(1<<num);
	//for(i=0;i<4;i++)printk("%c", (rel_state&1<<i)?'#':'_');printk("\n");
	i2c_bloc(SLAVE_ADDR(p_addr,WR),MODIO_ADDR_REL,sizeof(rel_state),&rel_state);
}
void set_addr(unsigned char addr){
	//printk(KERN_ALERT "new addr = %i\n",addr);
	i2c_bloc(SLAVE_ADDR(p_addr,WR),MODIO_ADDR_CHG,sizeof(addr),&addr);
	p_addr=addr;
}
