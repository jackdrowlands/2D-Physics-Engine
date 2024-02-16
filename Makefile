
CXX = g++
CXXFLAGS = -std=c++11 -Wall -I include
LDFLAGS = -lgtest -lgtest_main -pthread

TEST_SOURCES = $(wildcard *_test.cpp)
TEST_OBJECTS = $(TEST_SOURCES:.cpp=.o)
EXECUTABLE = run_tests

all: $(EXECUTABLE)

$(EXECUTABLE): $(TEST_OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *.o $(EXECUTABLE)

.PHONY: all clean
