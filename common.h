#ifndef COMMON_H
#define COMMON_H

#define MY_NAME "REMI2C_"
#define MY_PATH "/dev/"MY_NAME
#define MY_MAGIC 'x'
#define COUNT(array) (sizeof(array)/sizeof((array)[0]))
#define NAME(i) DevType[Devices[i].type],Devices[i].num

char*DevType[]={"REL","OPT","ANL"};
typedef enum   { REL , OPT , ANL }DeviceType;
typedef enum   { WR  , RD }DeviceAccess;
typedef enum   { OFF , ON }DeviceState;
typedef enum   { SET_ADDR }DeviceIoctlCmd;
typedef struct{
	DeviceType type;
	int num;
} Device_t ;

Device_t Devices[]={
	{REL,0},
	{REL,1},
	{REL,2},
	{REL,3},
	{OPT,0},
	{OPT,1},
	{OPT,2},
	{OPT,3},
	{ANL,0},
	{ANL,1},
	{ANL,2},
	{ANL,3}
};

#define MODIO_ID           0x58

#define MODIO_ADDR_REL     0x10
#define MODIO_ADDR_OPT     0x20
#define MODIO_ADDR_ANL     0x30
#define MODIO_ADDR_CHG     0xF0
//
#define SDA_DAT 21
#define SCL_DAT 20
#define SDA_CFG 20
#define SCL_CFG 16
//liste des commandes
#define I2C_READ_AIN(N)    0x30+N

#define TWI_BASE 0x01C2AC00
#define TWI(N) (TWI_BASE+(N*0x400))
typedef struct{
	volatile unsigned addr,xaddr,data,cntr,stat,ccr,srst,efr,lcr,dvfs;
}Twi_reg_t;

// PIO structure
#define PIO_BASE(PORT) 0x01C20800+(PORT*sizeof(Pio_reg_t))
#define NB_PORTS 9//length of Pio_ports_t
typedef enum{PA,PB,PC,PD,PE,PF,PG,PH,PI}Pio_ports_t;
typedef enum{IN,OUT}Pio_way_t;
typedef enum{LOW,HIGH}Pio_level_t;
typedef enum{ACK,NACK}Pio_ack_t;
typedef struct{
	volatile unsigned cfg0,cfg1,cfg2,cfg3,data,pad0,pad1,pad2,pad3;
}Pio_reg_t;
#endif