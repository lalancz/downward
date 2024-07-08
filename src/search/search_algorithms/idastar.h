#ifndef SEARCH_ALGORITHMS_IDASTAR_H
#define SEARCH_ALGORITHMS_IDASTAR_H

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

namespace idastar {

constexpr int AUX_SOLVED = -1;

class IDAstar : public SearchAlgorithm {
    int search_bound;
    const plugins::Options opts;
    
    int num_of_iterations;
    double total_iteration_times;
    int total_iteration_budgets;
    int total_nodes_expanded_per_iteration;

    int nodes;

    std::shared_ptr<Evaluator> f_evaluator;

    std::vector<OperatorID> operatorPath;
    std::vector<State> solutionPath;

    int search(std::vector<OperatorID> &operatorPath, std::vector<State> &solutionPath, int pathCost, 
        State currState, int bound, SearchStatistics &idastar_statistics);
    bool pathContains(std::vector<State> &path, State state);

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

public:
    explicit IDAstar(const plugins::Options &opts);
    virtual ~IDAstar() = default;

    virtual void print_statistics() const override;
};

extern void add_options_to_feature(plugins::Feature &feature);
}

#endif
