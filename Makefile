
CXX = g++ -g -std=c++11

all: lib test

lib: mapping.h mapping.cpp analyzer.h analyzer.cpp
	$(CXX) -c analyzer.cpp
	$(CXX) -c mapping.cpp

client: lib util.h plotter.h plotter.cpp main.cpp
	$(CXX) -c plotter.cpp
	$(CXX) -c main.cpp
	$(CXX) -o main main.o analyzer.o plotter.o mapping.o

test: lib test.cpp
	$(CXX) -c test.cpp
	$(CXX) -o test test.o analyzer.o mapping.o

run: all
	./test

java: test.java mapping.java
	javac test.java
	javac mapping.java

runjava: java
	java test