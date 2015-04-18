Travail effectué
================

- [Source](https://github.com/yne/modio_driver)
- [Archive](https://github.com/yne/modio_driver/archive/gh-pages.zip)

Gestion des entrées / sorties
--------

- lire les états des relais (4 devices REL0 à REL3). **OK**
- écrire dans les relais (0 ouvre et 1 ferme le relai). **OK**
- lire les entrées optocouplées (4 devices OPT0 à OPT3). **OK**
- lire les entrées analogiques (4 devices ANL0 à ANL3). **OK**

Modification de l'adresse
---------

- permettre la prise en charge d'adresses différentes. **OK**
- L'adresse à utiliser sera reçu en paramètre au chargement du module. **OK**
- Si aucun module ne répond à cette adresse, une commande de changement d'adresse sera envoyée à l'adresse par défaut (0x58) pour la modifier en la nouvelle adresse. **KO**

Cross compilation
=================

Afin de pouvoir exécuter le module sur notre cible (olimex A20)
il faut mettre en place plusieurs choses :

[compilation du noyau](build_kernel.sh)
--------------------
Cette étape a été la plus complexe car elle nécessite le bon déroulement de plusieurs processus qui dépendent parfois d'utilitaires qui n'étaient pas présent sur les machines (makeinfo).
Voici le script utilisé pour compiler le noyau ainsi que les outils de cross compilation.
On obtient en sortie, un fichier image (.img) contenant un linux complet (Kernel,binutils,environnement graphique) a flasher sur la SDcard.

On commence tout d'abord par récupérer l'ensemble des sources depuis différentes sources :
```
wget http://ftp.gnu.org/gnu/gcc/gcc-4.9.2/gcc-4.9.2.tar.gz              -q -O - | tar xz &
wget http://ftp.gnu.org/gnu/binutils/binutils-2.24.tar.gz               -q -O - | tar xz &
wget ftp://sourceware.org/pub/newlib/newlib-2.1.0.tar.gz                -q -O - | tar xz &
wget http://www.mpfr.org/mpfr-current/mpfr-3.1.2.tar.gz                 -q -O - | tar xz &
wget http://gmplib.org/download/gmp/gmp-6.0.0a.tar.xz                   -q -O - | tar xJ &
wget http://ftp.gnu.org/gnu/mpc/mpc-1.0.2.tar.gz                        -q -O - | tar xz &
wget http://ftp.gnu.org/gnu/gdb/gdb-7.8.tar.gz                          -q -O - | tar xz &
wget http://github.com/linux-sunxi/linux-sunxi/archive/sunxi-3.4.tar.gz -q -O - | tar xz &
wget http://github.com/linux-sunxi/u-boot-sunxi/archive/sunxi.tar.gz    -q -O - | tar xz && mv u-boot* uboot &
wget http://github.com/linux-sunxi/sunxi-tools/archive/master.tar.gz    -q -O - | tar xz && mv sunxi-tools* tools &
wget http://github.com/linux-sunxi/sunxi-boards/archive/master.tar.gz   -q -O - | tar xz && mv sunxi-boards* board &
wait
```
Chaque téléchargement est fait en parallèle et est décompressé à la volé (pour éviter de se retrouver avec le double de la taille des sources sur le disque à un instant T)

J'ai dû par la suite, patcher le makefile de la newlib car cette dernière avait une variable `tooldir` qui pointait vers un mauvais chemin :
```
sed -i 's|^tooldir = .*$|tooldir = $(exec_prefix)/$(target_alias)\nhost_makefile_frag = $(srcdir)/../../config/default.mh\n|g' $PREFIX_SRC/newlib/libgloss/arm/cpu-init/Makefile.in
```

Puis on compile successivement :
* binutils : qui est un ensemble d'outils de développement logiciel
	
	```
	cd $PREFIX_BUILD/binutils
	$PREFIX_SRC/binutils/configure -q --prefix=$PREFIX_BUILD/binutils --target=$TARGET
	make all -s
	make install -s
	```

* gmp : qui est une libraire de calcul arithmétique
	
	```
	cd $PREFIX_BUILD/gmp
	$PREFIX_SRC/gmp/configure -q --prefix=$PREFIX_BUILD/gmp
	make all -s
	make install -s
	```
* mpfr : qui est une autre libraire de calcul multi precision
	
	```
	cd $PREFIX_BUILD/mpfr
	$PREFIX_SRC/mpfr/configure -q --prefix=$PREFIX_BUILD/mpfr --target=$TARGET \
	--with-gmp-include=$PREFIX_BUILD/gmp/include \
	--with-gmp-lib=$PREFIX_BUILD/gmp/lib
	make all -s
	make install -s
	```

* mpc : qui est une libraire de calcul haute precision
	
	```
	cd $PREFIX_BUILD/mpc
	$PREFIX_SRC/mpc/configure -q --prefix=$PREFIX_BUILD/mpc --target=$TARGET \
	--with-gmp-include=$PREFIX_BUILD/gmp/include \
	--with-gmp-lib=$PREFIX_BUILD/gmp/lib \
	--with-mpfr-include=$PREFIX_BUILD/mpfr/include \
	--with-mpfr-lib=$PREFIX_BUILD/mpfr/lib
	make all -s
	make install -s
	```
	
* gcc : le GNU Compiler of Compiler, qui est le compilateur C utilisé par la suite
	
	```
	cd $PREFIX_BUILD/gcc
	$PREFIX_SRC/gcc/configure -q --prefix=$PREFIX_BUILD/gcc --target=$TARGET \
	  --without-headers \
	  --with-newlib \
	  --with-gnu-as \
	  --with-gnu-ld \
	  --with-gmp=$PREFIX_BUILD/gmp \
	  --with-mpfr=$PREFIX_BUILD/mpfr \
	  --with-mpc=$PREFIX_BUILD/mpc
	make all-gcc -s
	make install-gcc -s
	```
	
* newlib : une libraire standard C conçue pour les systèmes embarqués
	
	```
	cd $PREFIX_BUILD/newlib
	$PREFIX_SRC/newlib/configure -q --prefix=$PREFIX_BUILD/newlib --target=$TARGET
	#disable etc building (wich require makeinfo that i don't have)
	sed -i 's|all-host: maybe-all-etc|#all-host: maybe-all-etc|g' $PREFIX_BUILD/newlib/Makefile
	make -s
	#disable etc install (since they don't haven't been built)
	sed -i 's|    maybe-install-etc|#maybe-install-etc|g' $PREFIX_BUILD/newlib/Makefile
	make install -s
	```
	
* gcc+newlib : compilation de GCC en utilisant la newlib
	
	```
	export LD_LIBRARY_PATH=$PREFIX_BUILD/gmp/lib:$PREFIX_BUILD/mpc/lib:$PREFIX_BUILD/mpfr/lib:$PREFIX_BUILD/newlib/arm-eabi/lib
	cd $PREFIX_BUILD/gcc
	$PREFIX_SRC/gcc/configure -q --prefix=$PREFIX_BUILD/gcc --target=$TARGET\
	  --without-headers \
	  --with-newlib \
	  --with-gnu-as \
	  --with-gnu-ld \
	  --with-gmp=$PREFIX_BUILD/gmp \
	  --with-mpfr=$PREFIX_BUILD/mpfr \
	  --with-mpc=$PREFIX_BUILD/mpc \
	  --enable-languages=c,c++ \
	  --disable-shared \
	  --disable-libssp \
	  --disable-nls
	make all -s
	make install-gcc -s
	```
	
* gdb : le debuger de GNU
	
	```
	cd $PREFIX_BUILD/gdb
	rm -rf *
	$PREFIX_SRC/gdb/configure -q --prefix=$PREFIX_BUILD/gdb --target=$TARGET\
	  --enable-sim-arm\
	  --enable-sim-stdio
	make -s
	make install
	```
	
* kernel : le noyeau linux lui même
	
	```
	cd $PREFIX_SRC/linux
	make ARCH=arm CROSS_COMPILE=arm-eabi- sun5i_defconfig
	make ARCH=arm CROSS_COMPILE=arm-eabi- uImage modules -j4
	make ARCH=arm CROSS_COMPILE=arm-eabi- INSTALL_MOD_PATH=$PREFIX_BUILD modules_install
	```
	
* Uboot : l'utilitaire de boot utilisé par les olimex
	
	```
	cd $PREFIX_SRC/uboot
	make CROSS_COMPILE=arm-eabi- A13-OLinuXino_config 
	make CROSS_COMPILE=arm-eabi-
	```

Cependant, personne n'avons pas réussi a faire booter l'A13.
J'ai donc travaillé sur l'A20 pour la suite en utilisant la toolchain d'un étudiant qui avait commencé pour l'A20.

Implémentation du driver
========================

Pour mener à bien l'implémentation, j'ai séparer le code en 4 parties :

### [Partie GPIO](driver.pio.c)

Cette partie contient les fonctions d'accès basiques au PIO :

* pio_io : met un port en input ou en output
	
	```
	void pio_io(int port,Pio_way_t way){
		pioB->cfg2 &= ~(0b111 << port);//mise a 0 des 3 bits du port
		if(way==OUT)
			pioB->cfg2 |= (1 << port);//remise a 1 si en mode out
		pio_wait();
	}
	```
	
* pio_set : envoie une valeur sur le port
	
	```
	void pio_set(int port,Pio_level_t lv){//SDA_DAT,SCL_DAT
		if(lv==HIGH)pioB->data |= (1<< port);
		if(lv==LOW )pioB->data &=~(1<< port);
		pio_wait();
	}	
	```
	
* pio_get : récupère une valeur sur le port
	
	```
	unsigned char pio_get(int port){
		return !!(pioB->data & (1<< port));
	}
	```
	
* pio_ack : envoie un acquittement sur le port
	
	```
	void pio_ack(Pio_ack_t type){
		pio_io (SDA_CFG,OUT);
		pio_set(SDA_DAT,type);
		pio_set(SCL_DAT,HIGH);
		pio_set(SCL_DAT,LOW);
	}
	```

Ainsi que 2 fonctions de plus haut niveau qui offrent une interface de lecture/écriture sur le PIO.

* pio_recv : récupère 8 bits de donnée depuis le bus.
	
	```
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
	```
	
* pio_send : envoie 8 bits de données sur le bus.
	
	```
	void pio_send(unsigned char data){
		int i;
		for(i=7;i > =0;i--){
			pio_set(SDA_DAT,data & (1 << i) ? HIGH : LOW);
			pio_set(SCL_DAT,HIGH);
			pio_set(SCL_DAT,LOW);
		}
	}
	```
	

Enfin les 2 fonctions d'initialisation et de finalisation :

* pio_start : map l'adresse du GPIO dans une adresse virtuelle via `ioremap`
	
	```
	void pio_start(void){
		pioB = ioremap(PIO_BASE(PB),sizeof(Pio_reg_t));
		pio_io(SDA_CFG,OUT);
		pio_io(SCL_CFG,OUT);
	}
	```
	
* pio_stop : libère l'adresse virtuel avec `iounmap`
	
	```
	void pio_stop(void){
		iounmap(pioB);
	}
	```

### [Partie I2C](driver.pio.c)

Cette partie contient l'implémentation du protocole i2c.

* i2c_wait_ack  : Lis l'acquittement i2c
	
	```
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
	```
	
	![](https://upload.wikimedia.org/wikipedia/commons/d/d8/I2C_ACK.svg)
* i2c_sendStart : Envoie la trame de start
	
	```
	void i2c_sendStart(void){
		pio_set(SDA_DAT,HIGH);
		pio_set(SCL_DAT,HIGH);
		pio_set(SDA_DAT,LOW);
		pio_set(SCL_DAT,LOW);
	}
	```
	
	![](https://upload.wikimedia.org/wikipedia/commons/e/e8/I2C_START.svg)
* i2c_sendStop : Envoie la trame de stop
	
	```
	void i2c_sendStop(void){
		pio_set(SDA_DAT,LOW);
		pio_set(SCL_DAT,HIGH);
		pio_set(SDA_DAT,HIGH);
	}
	```
	
	![](https://upload.wikimedia.org/wikipedia/commons/b/b9/I2C_STOP.svg)
* i2c_send : Envoie une donnée sur le bus
	
	```
	void i2c_send(unsigned char data){
		pio_send(data);
		i2c_wait_ack();
	}
	```
	
	![](https://upload.wikimedia.org/wikipedia/commons/4/4d/I2C_Adresse10bitsEcriture.svg)

Je me suis créé une fonction dans le but d'alléger les couches supérieurs de la partie protocolaire (envoi du Start, envoi de l'ack ... envoi du stop)
* i2c_bloc : en charge de l'envoi ou de réception d'une trame i2c entière.
	
	```
	void i2c_bloc(unsigned char addr,unsigned char cmd,int size,unsigned char*data){
		int i;
		i2c_sendStart();
		i2c_send(addr);
		if((addr&1)==WR)i2c_send(cmd);
		for(i=0;i < size;i++){
			if((addr&1)==RD)data[i]=pio_recv(i < size-1?ACK:NACK);//ACK ACK ACK... NACK
			if((addr&1)==WR)i2c_send(data[i]);
		}
		i2c_sendStop();
	}
	```

### [Partie applicative](driver.app.c)

La partie applicative fait appel aux fonctions i2c et fournie une sur-couche simple pour le driver :

* read_opt : Permet de lire l'état d'un optocoupleur
	
	```
	unsigned char read_opt(int num){
		unsigned char states;
		i2c_bloc(SLAVE_ADDR(p_addr,WR),MODIO_ADDR_OPT,0,NULL);
		i2c_bloc(SLAVE_ADDR(p_addr,RD),MODIO_ADDR_OPT,sizeof(states),&states);
		return !!(states&(1 << num));
	}
	```
	
* read_anl : Lire sur l'entrée analogique
	
	```
	unsigned char read_anl(int num){
		unsigned char i,res=0,val[2];//low,high
		i2c_bloc(SLAVE_ADDR(p_addr,WR),MODIO_ADDR_ANL+num,0,NULL);
		i2c_bloc(SLAVE_ADDR(p_addr,RD),MODIO_ADDR_ANL+num,sizeof(val),val);
		//inverse les bits 0 a 9
		for(i=0;i< 8;i++){//sur les 8 premiers
			res |= (!!(val[0] & 0x80)) << i;
			val[0] <<= 1;
		}
		//et sur les 2 derniers
		res |= (!!(val[1] & 0b10)) << 8;
		res |= (!!(val[1] & 0b01)) << 9;
	
		return res>>2;
	}
	```
	
* write_rel : Active/désactive un relais et met a jours le cache d'état des relais.
	
	```
	void write_rel(unsigned char num,Pio_level_t lv){
		if(lv==HIGH)rel_state |=  (1<< num);
		if(lv==LOW )rel_state &= ~(1<< num);
		//for(i=0;i< 4;i++)printk("%c", (rel_state&1<< i)?'#':'_');printk("\n");
		i2c_bloc(SLAVE_ADDR(p_addr,WR),MODIO_ADDR_REL,sizeof(rel_state),&rel_state);
	}
	```
	
* read_rel : permet de lire l'état d'un relais à partir du cache d'état des relais.
	
	```
	unsigned char read_rel(unsigned char num){
		return (rel_state >>num)&1;
	}
	```
	
* rel_state : permet de connaitre l'état des relais afin d'en renvoyer l'état si demandé
	
	```
	unsigned char rel_state=0;
	```
	
* set_addr : permet de changer l'adresse du modio
	
	```
	void set_addr(unsigned char addr){
		i2c_bloc(SLAVE_ADDR(p_addr,WR),MODIO_ADDR_CHG,sizeof(addr),&addr);
		p_addr=addr;
	}
	```

Enfin, pour récupérer l'adresse demandée par l'utilisateur au lancement du module j'utilise :
```
static int p_addr=MODIO_ID;
module_param(p_addr, int, S_IRUGO);
```

### [Partie driver (File Operations)](driver.fop.c)

Cette partie est la couche de plus haut niveau, elle fournie les interfaces avec le kernel pour répondre aux appels open/read/write/close de l'utilisateur.

* my_open : met à jours les pointeurs vers les fonctions de read et de write selon le type de périphérique demandé (car certain périphérique sont en lecture seulement)
	
	```
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
	```
	
* my_read : se base sur le mineur (0~11) pour savoir quel est le type de périphérique visé et appel la fonction associée (read_opt, read_anl, read_rel)
	
	```
	ssize_t my_read(struct file *f, char *buf, size_t size, loff_t *offset){
		int i,minor=iminor(f->f_path.dentry->d_inode);
		unsigned char val=0,num=Devices[minor].num;
		
		printk(KERN_ALERT "%s(%s%i)\n",__FUNCTION__,NAME(minor));
		for(i=0;i < size;i++){
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
	```
	
* my_write : appel la fonction d'écriture associée au type de device (donc seulement write_rel puisque il n'y a que le relais qui supporte l'écriture)
	
	```
	ssize_t my_write(struct file *f, const char *buf, size_t size, loff_t *offset){
		int i,minor=iminor(f->f_path.dentry->d_inode);
		char val=0;
		
		for(i=0;i < size;i++){
			if(copy_from_user(&val, buf+i, sizeof(val)))break;
			printk(KERN_ALERT "%s(%s%i,%i)\n",__FUNCTION__, NAME(minor),val);
			switch(Devices[minor].type){
				case REL:write_rel(Devices[minor].num,val?HIGH:LOW);break;
				default:break;
			}
		}
		return i;
	}
	```
	
* my_ioctl : permet de faire des action spéciales tel qu'un changement d'adresse.
	
	```
	long my_ioctl(struct file *f, unsigned int cmd, unsigned long arg){
		int minor=iminor(f->f_path.dentry->d_inode);
		printk(KERN_ALERT "%s(%s%i,%i,%lu)\n",__FUNCTION__, NAME(minor),cmd,arg);
		switch(cmd){
			case SET_ADDR:set_addr(arg);break;
			default:break;
		}
		return 0;
	}
	```
	
* my_close : ne fait rien.
	
	```
	int my_close(struct inode *in, struct file *f){
		printk(KERN_ALERT "%s(%s%i)\n",__FUNCTION__,NAME(iminor(in)));
		return 0;
	}
	```
	

### Partie driver (Module)

Cette partie initialise et finalise le module

* init_module : appel la fonction d'initialisation du pio, crée les cdev,les classes et les devices.
	
	```
	int init_module(void){
		int i;
		pio_start();
		
		if (alloc_chrdev_region(&dev,0,COUNT(Devices),MY_NAME"dev") < 0)
			return printk(KERN_ALERT "alloc_chrdev_region error\n");//cat /proc/devices
		
		my_cdev = cdev_alloc();
		my_fops=(struct file_operations){.owner = THIS_MODULE,
			.open = my_open,.release = my_close,.unlocked_ioctl = my_ioctl};
		my_cdev->ops = &my_fops;
		cdev_add(my_cdev,dev,COUNT(Devices)); 
		my_class = class_create(THIS_MODULE , MY_NAME"class"); //ls /sys/class
		
		for(i=0;i < COUNT(Devices);i++)//ls /dev/
			device_create(my_class, NULL, MKDEV(MAJOR(dev), MINOR(dev)+i), NULL,
				MY_NAME"%s%i",DevType[Devices[i].type],Devices[i].num);
		
		return 0;
	}
	```
	
* cleanup_module : des-alloues les cdev,les classes et les devices et referme le PIO
	
	```
	void cleanup_module(void){
		int i;
		for(i=0;i < 12;i++)
			device_destroy(my_class, MKDEV(MAJOR(dev), MINOR(dev)+i));
		class_destroy(my_class);
		unregister_chrdev_region(dev,COUNT(Devices));
		cdev_del(my_cdev);
		pio_stop();
	}
	```

### [Programme de test](bench.c)

Ce programme de test permet de vérifier si le driver répond correctement aux requêtes d'accès de l'utilisateur.
Il appel donc les différentes fonctions du driver (open/read/write/close/ioctl)
Il permet aussi de lister les périphériques disponibles.

La fonction main utilise les arguments reçus en paramètres (argv)
pour savoir l'enchainement des opérations à effectuer.
Il est donc possible de l'appeler de la sorte :
```
./bench open 1 read close open 2 read write 1 close
```

ou bien des scripts plus complexes

```
dmesg -c > /dev/null
insmod remy_driver.ko
gcc bench.c -std=c99 -o bench

# chenillard
./bench \
	open 0 write 1 close sleep 250\
	open 0 write 0 close sleep 250\
	open 1 write 1 close sleep 250\
	open 2 write 1 close sleep 250\
	open 3 write 1 close sleep 250\
	open 0 write 0 close sleep 250\
	open 1 write 0 close sleep 250\
	open 2 write 0 close sleep 250\
	open 3 write 0 close sleep 250\
	open 0 write 1 close sleep 250\
	open 1 write 1 close sleep 250\
	open 2 write 1 close sleep 250\
	open 3 write 1 close sleep 250\
	open 0 write 0 close sleep 250\
	open 1 write 0 close sleep 250\
	open 2 write 0 close sleep 250\
	open 3 write 0 close sleep 250

#./bench \
#	open 1 ioctl 0 42 close sleep 250\
#	open 1 write 1 close sleep 250\
#	open 1 write 0 close sleep 250



rmmod remy_driver.ko
dmesg
```

Le fichier [`bench.sh`](bench.sh) regroupe la compilation du [`bench.c`](bench.c) et son lancement.

Documents utilisés
==================

* http://linux-sunxi.org/Linux_Kernel
* https://www.olimex.com/Products/Modules/IO/MOD-IO/open-source-hardware
* https://github.com/allwinner-zh/documents/blob/master/A20/A20%20user%20manual%20v1.3%2020141010.pdf?raw=true
* http://www.py6zgp.com/download/A20-GPIO.pdf
* http://lwn.net/images/pdf/LDD3/ch09.pdf
* https://www.olimex.com/Products/Modules/IO/MOD-IO/resources/MOD-IO.pdf
