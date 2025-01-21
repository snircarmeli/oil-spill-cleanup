# Compiler flags
CXXFLAGS = -Wall -g -std=c++17 -I/usr/include/eigen3 -I/usr/include/json

# Object files
OBJS = main.o generic-boat.o boom-boat.o boom-boats-duo.o
RM = rm -rf

# Executable name
EXEC = main.exe

# Default target
all: $(EXEC)

# Link object files to create the executable
$(EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Compile individual .o files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean target to remove object files and executable
clean:
	$(RM) $(OBJS)

# Mark "all" and "clean" as phony targets
.PHONY: all clean