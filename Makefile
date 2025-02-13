CXX := g++
CXXFLAGS := -Wall -Wextra -fopenmp -pedantic -std=c++20
RELEASEFLAGS := -O3

# List of source files
SRCS := io.cc simulate.cc
HEADERS := io.h collision.h sim_validator.h

# Object files
OBJS := $(SRCS:.cc=.o)

.PHONY: all clean

all: release

# List of executables (actual binary names)
EXECUTABLES := sim
PERF_EXECUTABLES := $(EXECUTABLES:%=%.perf)

TARGETS := $(EXECUTABLES) 
PERF_TARGETS := $(PERF_EXECUTABLES)

release: $(TARGETS) $(PERF_TARGETS)

# How to compile .o and .o.perf object files
%.o: %.cc $(HEADERS)
	$(CXX) $(CXXFLAGS) -DCHECK=1 $(RELEASEFLAGS) -c $< -o $@
%.o.perf: %.cc $(HEADERS)
	$(CXX) $(CXXFLAGS) -DCHECK=0 $(RELEASEFLAGS) -c $< -o $@

# How to compile non-perf and perf executables
$(EXECUTABLES): %: %.o io.o sim_validator.a simulate.o
	$(CXX) $(CXXFLAGS) -DCHECK=1 $(RELEASEFLAGS) -o $@ $^
$(PERF_EXECUTABLES): %.perf: %.o.perf io.o simulate.o
	$(CXX) $(CXXFLAGS) -DCHECK=0 $(RELEASEFLAGS) -o $@ $^

clean:
	$(RM) *.o *.o.perf $(EXECUTABLES) $(PERF_EXECUTABLES)
