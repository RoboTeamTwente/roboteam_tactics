CC=g++
CFLAGS=-I../../include -std=c++11

%.o: %.cpp
	$(CC) -c -o $@ $< $(CFLAGS)

converter: converter.o BTBuilder.o
	g++ converter.o BTBuilder.o -o converter 
