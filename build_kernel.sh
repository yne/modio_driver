#/bin/sh

export PREFIX=$HOME/Documents/TP_Compile
export PREFIX_BUILD=$PREFIX/build
export PREFIX_SRC=$PREFIX/sources
# on reconfigure le path pour aller récupérer les binaire dans NOS dossiers en premier
export PATH="$PREFIX/bin:$PATH"
# export PATH="$PREFIX/bin:~/bin:$PATH"
export TARGET=arm-eabi

mkdir -p $PREFIX
mkdir -p $PREFIX_SRC
mkdir -p $PREFIX_BUILD

###########
# SOURCES #
###########
cd $PREFIX_SRC

echo "Recuperation des sources"
# arguments tar = x:eXtract, z:gz, j:.bz2, J:xz
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

echo "Retrait des numeros de versions."
for i in * ; do mv "$i" "${i/-*/}" ; done

echo "patch le Makefile de la newLib"
cp $PREFIX_SRC/newlib/libgloss/arm/cpu-init/Makefile.in $PREFIX_SRC/newlib/libgloss/arm/cpu-init/Makefile.in.old
sed -i 's|^tooldir = .*$|tooldir = $(exec_prefix)/$(target_alias)\nhost_makefile_frag = $(srcdir)/../../config/default.mh\n|g' $PREFIX_SRC/newlib/libgloss/arm/cpu-init/Makefile.in
sed -i 's|@host_makefile_frag_path@|${host_makefile_frag_path}|g' $PREFIX_SRC/newlib/libgloss/arm/cpu-init/Makefile.in

###################
#   COMPILATIONS  #
###################

echo "Creation des dossiers de build."
(cd $PREFIX_SRC;for i in * ; do
	mkdir -p "$PREFIX_BUILD/$i";
done)

##########################
# compilation des binutils 
##########################

cd $PREFIX_BUILD/binutils
$PREFIX_SRC/binutils/configure -q --prefix=$PREFIX_BUILD/binutils --target=$TARGET
make all -s
make install -s

#####################
# compilation de gmp
#####################

cd $PREFIX_BUILD/gmp
$PREFIX_SRC/gmp/configure -q --prefix=$PREFIX_BUILD/gmp
make all -s
make install -s

#####################
# compilation de mpfr
#####################

cd $PREFIX_BUILD/mpfr
$PREFIX_SRC/mpfr/configure -q --prefix=$PREFIX_BUILD/mpfr --target=$TARGET \
--with-gmp-include=$PREFIX_BUILD/gmp/include \
--with-gmp-lib=$PREFIX_BUILD/gmp/lib
make all -s
make install -s

#####################
# compilation de mpc
#####################

cd $PREFIX_BUILD/mpc
$PREFIX_SRC/mpc/configure -q --prefix=$PREFIX_BUILD/mpc --target=$TARGET \
--with-gmp-include=$PREFIX_BUILD/gmp/include \
--with-gmp-lib=$PREFIX_BUILD/gmp/lib \
--with-mpfr-include=$PREFIX_BUILD/mpfr/include \
--with-mpfr-lib=$PREFIX_BUILD/mpfr/lib
make all -s
make install -s

#####################
# compilation de gcc
#####################

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

###########################
# compilation de la newlib
###########################

cd $PREFIX_BUILD/newlib
$PREFIX_SRC/newlib/configure -q --prefix=$PREFIX_BUILD/newlib --target=$TARGET
#disable etc building (wich require makeinfo that i don't have)
sed -i 's|all-host: maybe-all-etc|#all-host: maybe-all-etc|g' $PREFIX_BUILD/newlib/Makefile
make -s
#disable etc install (since they don't haven't been built)
sed -i 's|    maybe-install-etc|#maybe-install-etc|g' $PREFIX_BUILD/newlib/Makefile
make install -s

################################
# compilation de GCC avec newlib
################################

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

#########################
# compilation de la gdb
#########################

cd $PREFIX_BUILD/gdb
rm -rf *
$PREFIX_SRC/gdb/configure -q --prefix=$PREFIX_BUILD/gdb --target=$TARGET\
  --enable-sim-arm\
  --enable-sim-stdio
make -s
make install

############################
# compilation du kernel A13
############################

cd $PREFIX_SRC/linux
make ARCH=arm CROSS_COMPILE=arm-eabi- sun5i_defconfig
make ARCH=arm CROSS_COMPILE=arm-eabi- uImage modules -j4
make ARCH=arm CROSS_COMPILE=arm-eabi- INSTALL_MOD_PATH=$PREFIX_BUILD modules_install

############################
# compilation de Uboot
############################

cd $PREFIX_SRC/uboot
make CROSS_COMPILE=arm-eabi- A13-OLinuXino_config 
make CROSS_COMPILE=arm-eabi-

#sed -i 's|all-host: maybe-all-texinfo|A13-OLinuXino|g' boards.cfg
#grep sunxi boards.cfg | awk '{print $7}'
#target -> A13-OLinuXino


#script.bin : http://linux-sunxi.org/Manual_build_howto

cd $PREFIX_SRC/tools
make fex2bin
./fexc $PREFIX_SRC/board/sys_config/a13/a13-olinuxino.fex script.bin