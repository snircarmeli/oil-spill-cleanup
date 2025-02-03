# Compiler flags
CXXFLAGS = -Wall -g -std=c++17 -I/usr/include/json -I/usr/include/eigen3

# Object files
OBJS = main_dubin_check.o dubin.o generic-boat.o boom-boat.o boom-boats-duo.o integrator.o dubin.o PID_controller.o helper_funcs.o oil-spill.o
RM = rm -rf

# Executable name
EXEC = main_dubin_check.exe # main.exe

# Default target
all: $(EXEC)

# Link object files to create the executable
$(EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Compile individual .o files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Ensure that boom-boats-duo.o knows it needs integrator.h
# boom-boats-duo.o: boom-boats-duo.cpp integrator.h
# 	$(CXX) $(CXXFLAGS) -c boom-boats-duo.cpp -o boom-boats-duo.o

# Clean target to remove object files and executable
clean:
	$(RM) $(OBJS)

# Mark "all" and "clean" as phony targets
.PHONY: all clean