bitspi.o: bitspi.c bitspi.h
	gcc -Wall -c bitspi.c

rfid.o: rfid.c rfid.h
	gcc -Wall -c rfid.c

test: test.c rfid.o bitspi.o
	gcc -Wall test.c rfid.o bitspi.o -lwiringPi -o test

clean:
	rm *.o
	rm test
