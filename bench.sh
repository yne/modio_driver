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
