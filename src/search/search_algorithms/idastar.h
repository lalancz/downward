#ifndef SEARCH_ALGORITHMS_IDASTAR_H
#define SEARCH_ALGORITHMS_IDASTAR_H

#include "../open_list.h"
#include "../search_algorithm.h"

#include "idastar_aux.h"

#include <memory>
#include <vector>
#include <stack>

class Evaluator;
class PruningMethod;

namespace plugins {
class Feature;
}

namespace idastar {
class IDAstar : public SearchAlgorithm {
    const bool reopen_closed_nodes;

    std::unique_ptr<StateOpenList> open_list;
    std::stack<StateID> path;
    std::shared_ptr<Evaluator> f_evaluator;

    std::vector<Evaluator *> path_dependent_evaluators;
    std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
    std::shared_ptr<Evaluator> lazy_evaluator;

    std::shared_ptr<PruningMethod> pruning_method;

    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(EvaluationContext &eval_context);
    void reward_progress();

    idastar_aux::IDAstar_aux idastar_aux;

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

public:
    explicit IDAstar(const plugins::Options &opts);
    virtual ~IDAstar() = default;

    virtual void print_statistics() const override;

    void dump_search_space() const;
};

extern void add_options_to_feature(plugins::Feature &feature);
}

#endif
