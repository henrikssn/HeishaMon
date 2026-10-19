CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -I../HeishaMon -I. -O2

COMMON_OBJS = mock_heishamon.o ../HeishaMon/meisha.o
TARGETS = test_ticket_01 test_ticket_02 test_ticket_03 test_ticket_04 test_ticket_05

all: $(TARGETS)

test_ticket_01: test_ticket_01.o $(COMMON_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

test_ticket_02: test_ticket_02.o $(COMMON_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

test_ticket_03: test_ticket_03.o $(COMMON_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

test_ticket_04: test_ticket_04.o $(COMMON_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

test_ticket_05: test_ticket_05.o $(COMMON_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

test: $(TARGETS)
	./test_ticket_01
	./test_ticket_02
	./test_ticket_03
	./test_ticket_04
	./test_ticket_05

clean:
	rm -f $(TARGETS) test_runner *.o ../HeishaMon/meisha.o

build:
	$(MAKE) -C .. build

build-binary:
	$(MAKE) -C .. build-binary

.PHONY: all test clean build build-binary
