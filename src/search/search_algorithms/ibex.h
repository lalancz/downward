#ifndef SEARCH_ALGORITHMS_IBEX_H
#define SEARCH_ALGORITHMS_IBEX_H

#include "../open_list.h"
#include "../search_algorithm.h"

#include "../plugins/options.h"

#include <memory>
#include <vector>
#include <stack>

class Evaluator;

namespace plugins {
class Feature;
}

namespace ibex {

class IBEX : public SearchAlgorithm {
    const bool force_idastar;

    int num_of_iterations;

    int search_bound;
    const plugins::Options opts;

    std::shared_ptr<Evaluator> evaluator;

    std::pair<int, int> interval_intersection(std::pair<int, int> i1, std::pair<int, int> i2);
    std::pair<int, int> search(int costLimit, int nodeLimit);
    void limitedDFS(State currState, int pathCost, int costLimit, int nodeLimit, std::vector<State> &currentPath,
        std::vector<OperatorID> &currentSolutionPath);
    bool pathContains(std::vector<State> &path, State state);

    bool check_goal();

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

public:
    Plan solutionPath;
    int solutionCost;
    int solutionLowerBound;

    int budget;
    int nodes;

    int c_1;
    int c_2;

    int f_below;
    int f_above;

    bool goalFoundCurrentIteration;

    std::pair<int, int> i;

    explicit IBEX(const plugins::Options &opts);
    virtual ~IBEX() = default;

    virtual void print_statistics() const override;
};

extern void add_options_to_feature(plugins::Feature &feature);
}

#endif
