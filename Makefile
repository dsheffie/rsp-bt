# rsp-bt: N64 RSP interpreter (rsp.cc) and LLVM binary translator (rspbt*)
LLVM_CONFIG ?= llvm-config
CXX ?= g++
CXXFLAGS = -std=c++17 -O2 -g -Wall
LLVM_CXXFLAGS = $(filter-out -std=%,$(shell $(LLVM_CONFIG) --cxxflags))

OBJ = obj/rsp.o obj/rspbt.o obj/rspFunc.o obj/rspInstruction.o

all: librspbt.a

librspbt.a: $(OBJ)
	ar rcs $@ $(OBJ)

obj/rsp.o: rsp.cc rsp.hh
	@mkdir -p obj
	$(CXX) $(CXXFLAGS) -c $< -o $@

obj/%.o: %.cc
	@mkdir -p obj
	$(CXX) $(CXXFLAGS) $(LLVM_CXXFLAGS) -MMD -c $< -o $@

-include obj/*.d

tests/rsp_difftest: tests/rsp_difftest.cc librspbt.a
	$(CXX) $(CXXFLAGS) $(LLVM_CXXFLAGS) -I. -o $@ $< librspbt.a $(shell $(LLVM_CONFIG) --ldflags --libs) -lpthread

clean:
	rm -rf obj librspbt.a tests/rsp_difftest
