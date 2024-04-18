#ifndef SEARCH_ALGORITHMS_IDASTAR_AUX__H
#define SEARCH_ALGORITHMS_IDASTAR_AUX_H

#include "../open_list.h"
#include "../search_algorithm.h"

#include <memory>
#include <vector>

class Evaluator;
class PruningMethod;

namespace plugins {
class Feature;
}

namespace idastar_aux {
class IDAstar_aux : public SearchAlgorithm {
    const bool reopen_closed_nodes;

    std::unique_ptr<StateOpenList> open_list;
    std::shared_ptr<Evaluator> f_evaluator;

    std::vector<Evaluator *> path_dependent_evaluators;
    std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
    std::shared_ptr<Evaluator> lazy_evaluator;

    std::shared_ptr<PruningMethod> pruning_method;

    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(EvaluationContext &eval_context);
    void reward_progress();

public:
    virtual void initialize() override;
    virtual SearchStatus step() override;
    
    explicit IDAstar_aux(const plugins::Options &opts);
    virtual ~IDAstar_aux() = default;

    virtual void print_statistics() const override;

    void dump_search_space() const;
};

extern void add_options_to_feature(plugins::Feature &feature);
}

#endif
