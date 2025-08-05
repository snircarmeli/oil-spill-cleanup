# Compiler flags
CXXFLAGS = -Wall -g -std=c++17 -I./json -I./eigen -I/home/snir-carmeli/Thesis_work/code/gurobi1202/linux64/include
# LDFLAGS = -L/home/snir-carmeli/Thesis_work/code/gurobi1202/linux64/lib -lgurobi_c++ -lgurobi120
LDFLAGS = -Wl,-rpath,/home/snir-carmeli/Thesis_work/code/gurobi1202/linux64/lib -L/home/snir-carmeli/Thesis_work/code/gurobi1202/linux64/lib -lgurobi_c++ -lgurobi120

# Object files
OBJS = main_dubin_check.o dubin.o generic-boat.o boom-boat.o boom-boats-duo.o integrator.o dubin.o helper_funcs.o oil-spill.o obstacle.o set_point_controller.o MILP_Allocator.o 
RM = rm -rf

# Executable name
EXEC = main_dubin_check.exe # main.exe 

# Default target
all: $(EXEC)

# Link object files to create the executable
$(EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
	
# Compile individual .o files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Ensure that boom-boats-duo.o knows it needs integrator.h
# boom-boats-duo.o: boom-boats-duo.cpp integrator.h
# 	$(CXX) $(CXXFLAGS) -c boom-boats-duo.cpp -o boom-boats-duo.o

# Clean target to remove object files and executable
clean:
	$(RM) $(OBJS) $(EXEC)

# Mark "all" and "clean" as phony targets
.PHONY: all clean