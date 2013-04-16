run: dSPIN_run.o dSPIN.h dSPIN_commands.o dSPIN_support.o
	g++ -o run dSPIN_run.o dSPIN_commands.o dSPIN_support.o -l wiringPi
dSPIN_run.o: dSPIN.h dSPIN_commands.o dSPIN_support.o
	g++ -c dSPIN_run.c
test: test_alpha dSPIN_test.o dSPIN.h dSPIN_commands.o dSPIN_support.o
	g++ -o test dSPIN_test.o dSPIN_commands.o dSPIN_support.o -l wiringPi
dSPIN_test.o: dSPIN.h dSPIN_commands.o dSPIN_support.o
	g++ -c dSPIN_test.c
dSPIN_commands.o: dSPIN.h dSPIN_support.o
	g++ -c dSPIN_commands.c
dSPIN_support.o: dSPIN.h
	g++ -c dSPIN_support.c
clean:
	rm *.o test
