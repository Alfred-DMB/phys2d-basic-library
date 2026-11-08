CXX = g++
CXXFLAGS = -std=c++23 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror -O2 -Iinclude

HEADERS := $(wildcard include/phys2d/*.hpp)

all: demo ex_stack ex_chain ex_raycast ex_vehicle tests_bin

demo: examples/main.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) examples/main.cpp -o demo

ex_stack: examples/ex_stack.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) examples/ex_stack.cpp -o ex_stack

ex_chain: examples/ex_chain.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) examples/ex_chain.cpp -o ex_chain

ex_raycast: examples/ex_raycast.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) examples/ex_raycast.cpp -o ex_raycast

ex_vehicle: examples/ex_vehicle.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) examples/ex_vehicle.cpp -o ex_vehicle

tests_bin: tests/tests.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) tests/tests.cpp -o tests_bin

bench: examples/bench.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) examples/bench.cpp -o bench

test: all
	./tests_bin
	./demo
	./ex_stack
	./ex_chain
	./ex_raycast
	./ex_vehicle

sanitizers: tests/tests.cpp $(HEADERS)
	$(CXX) -std=c++23 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -O1 -g -Iinclude tests/tests.cpp -o tests_san
	./tests_san

clean:
	rm -f demo ex_stack ex_chain ex_raycast ex_vehicle tests_bin bench tests_san

.PHONY: all test bench clean sanitizers
