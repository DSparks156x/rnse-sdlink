# host_link: the firmware's stream store (sdemu/avstream.cpp + sdproto.cpp, unchanged) on Linux/WSL with a pty
# standing in for the Teensy's USB CDC port. Used by ../../../pi/avlink/test/run_tests.py.
CXX ?= g++
CXXFLAGS ?= -O2 -g -std=c++17 -Wall -Wextra -Wno-unused-parameter -DAVS_HOST
SRC = main.cpp ../../sdemu/avstream.cpp ../../sdemu/sdproto.cpp

host_link: $(SRC) ../../sdemu/avstream.h ../../sdemu/sdproto.h
	$(CXX) $(CXXFLAGS) $(SRC) -o $@

clean:
	rm -f host_link
.PHONY: clean
