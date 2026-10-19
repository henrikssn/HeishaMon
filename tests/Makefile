CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -I../HeishaMon -I. -O2

SRCS = test_ticket_01.cpp mock_heishamon.cpp ../HeishaMon/meisha.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = test_runner

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) test_ticket_01.o mock_heishamon.o ../HeishaMon/meisha.o

.PHONY: all test clean
