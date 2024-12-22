# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -g -std=c++11 -I/usr/include/eigen3

# Object files
OBJS = main.o generic-boat.o
RM = rm -rf

# Executable name
EXEC = main.exe

# Default target
all: $(EXEC)

# Link object files to create the executable
$(EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Compile main.o
main.o: main.cpp generic-boat.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile generic-boat.o
generic-boat.o: generic-boat.cpp generic-boat.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean target to remove object files and executable
clean:
	$(RM) $(OBJS) $(EXEC)

# Mark "all" and "clean" as phony targets
.PHONY: all clean
