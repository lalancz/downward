#ifndef SEARCH_ALGORITHMS_IDASTAR_H
#define SEARCH_ALGORITHMS_IDASTAR_H

#include "../open_list.h"
#include "../search_algorithm.h"

#include "idastar_aux.h"
#include "../plugins/options.h"

#include <memory>
#include <vector>
#include <stack>

class Evaluator;

namespace plugins {
class Feature;
}

namespace idastar {

enum AuxSearchStatus {
    AUX_FAILED = -1,
    AUX_SOLVED = -2,
    AUX_IN_PROGRESS = -3
};

class IDAstar : public SearchAlgorithm {
    int search_bound;
    const plugins::Options opts;

    std::shared_ptr<Evaluator> f_evaluator;
    std::vector<StateID> path;

    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(EvaluationContext &eval_context);

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
