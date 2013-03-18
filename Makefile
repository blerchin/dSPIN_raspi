test: dSPIN_main.o dSPIN.h dSPIN_commands.o dSPIN_support.o
	g++ -o test dSPIN_main.o dSPIN_commands.o dSPIN_support.o -l wiringPi
dSPIN_main.o: dSPIN.h dSPIN_commands.o dSPIN_support.o
	g++ -c dSPIN_main.c
dSPIN_commands.o: dSPIN.h dSPIN_support.o
	g++ -c dSPIN_commands.c
dSPIN_support.o: dSPIN.h
	g++ -c dSPIN_support.c
clean:
	rm *.o test
