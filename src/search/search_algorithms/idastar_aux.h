#ifndef SEARCH_ALGORITHMS_IDASTAR_AUX_H
#define SEARCH_ALGORITHMS_IDASTAR_AUX_H

#include "../open_list.h"
#include "../search_algorithm.h"

#include <memory>
#include <vector>
#include <stack>

class Evaluator;

namespace plugins {
class Feature;
}

namespace idastar_aux {
class IDAstar_aux : public SearchAlgorithm {
    std::unique_ptr<StateOpenList> open_list;
    std::shared_ptr<Evaluator> evaluator;

    std::vector<Evaluator *> path_dependent_evaluators;

    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(EvaluationContext &eval_context);


public:
    std::vector<StateID> path;

    virtual void initialize() override;
    virtual SearchStatus step() override;
    virtual int search(std::vector<StateID> &path, int bound);

    virtual bool path_contains(std::vector<StateID> &path, StateID state) const;
    
    explicit IDAstar_aux(const plugins::Options &opts);
    virtual ~IDAstar_aux() = default;

    virtual void print_statistics() const override;

    void dump_search_space() const;
};

extern void add_options_to_feature(plugins::Feature &feature);
}

#endif
