CC=g++
FLAGS=-std=c++11 -Werror -Wsign-conversion

CPPs=Graph.cpp Algorithms.cpp TestCounter.cpp Test.cpp
OBJECTS=$(subst .cpp,.o,$(CPPs))

all: demo test

run: test
	./$^

demo: Demo.o $(filter-out TestCounter.o Test.o, $(OBJECTS))
	$(CC) $(FLAGS) $^ -o demo

test: TestCounter.o Test.o $(filter-out Demo.o, $(OBJECTS))
	$(CC) $(FLAGS) $^ -o test

%.o: %.cpp
	$(CC) $(FLAGS) --compile $< -o $@

clean:
	rm -f *.o demo test
