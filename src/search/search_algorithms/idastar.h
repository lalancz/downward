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

    int nodes;

    std::shared_ptr<Evaluator> f_evaluator;

    std::vector<OperatorID> operatorPath;

    int search(std::vector<OperatorID> &operatorPath, int pathCost, State currState, int bound);

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
