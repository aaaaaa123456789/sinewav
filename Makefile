all:
	nasm -felf64 sinewav.s -o sinewav.o
	ld -s -n sinewav.o -o sinewav

debug:
	nasm -felf64 -g sinewav.s -o sinewav.o
	ld -n sinewav.o -o sinewav

clean:
	rm -f sinewav.o sinewav
