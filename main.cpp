#include <iostream>
#include <vector>
#include <numeric>
#include <stack>
#include <cassert>
#include <memory>
#include "lib/refrain_solvers.h"

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    RefrainReader reader;
    reader.read();
    RefrainSuffixArraySolver arraySolver(reader);
    RefrainSuffixAutomatonSolver automatonSolver(reader);
    RefrainSuffixTreeSolver treeSolver(reader);
    arraySolver.solve();
    automatonSolver.solve();
    treeSolver.solve();
    assert(arraySolver == automatonSolver && automatonSolver == treeSolver);
    automatonSolver.printAns();
    return 0;
}