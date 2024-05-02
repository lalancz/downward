#include "idastar_aux.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../open_list_factory.h"
#include "../pruning_method.h"

#include "../algorithms/ordered_set.h"
#include "../plugins/options.h"
#include "../task_utils/successor_generator.h"
#include "../utils/logging.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <optional>
#include <set>
#include <stack>

using namespace std;

namespace idastar_aux {
IDAstar_aux::IDAstar_aux(const plugins::Options &opts)
    : SearchAlgorithm(opts),
      evaluator(opts.get<shared_ptr<Evaluator>>("eval", nullptr)) {
}

void IDAstar_aux::initialize() {
    return;
}

void IDAstar_aux::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
}

SearchStatus IDAstar_aux::step() {
    return SOLVED;
}

int IDAstar_aux::search(std::vector<StateID> &path, int bound, OpenList<StateID> *open_list) {
    optional<SearchNode> node;

    StateID id = open_list->remove_min();
    State state = state_registry.lookup_state(id);
    node.emplace(search_space.get_node(state));
    const State &s = node->get_state();
    if (check_goal_and_set_plan(s)){
        std::cout << "Found a solution with cost " << node->get_g() << endl;
    
        return SOLVED;
    }
    
    EvaluationContext eval_context(s, node->get_g(), false, &statistics);
    int h = eval_context.get_evaluator_value_or_infinity(evaluator.get());
    int f = node->get_g() + h;

    if (f > bound)
        return f;

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(s, applicable_ops);

    int next_bound = numeric_limits<int>::max();
    for (OperatorID op_id : applicable_ops) {
        OperatorProxy op = task_proxy.get_operators()[op_id];

        State succ_state = state_registry.get_successor_state(s, op);
        statistics.inc_generated();

        if (path_contains(path, succ_state.get_id()) != -1)
            continue;

        SearchNode succ_node = search_space.get_node(succ_state);
        int succ_g = node->get_g() + get_adjusted_cost(op);
        EvaluationContext succ_eval_context(succ_state, succ_g, false, &statistics);
        statistics.inc_evaluated_states();

        succ_node.open(*node, op, get_adjusted_cost(op));
        open_list->insert(succ_eval_context, succ_state.get_id());
        path.push_back(succ_state.get_id());

        int t = search(path, bound, open_list);
        if (t == SOLVED) {
            return SOLVED;
        } else if (t < next_bound) {
            next_bound = t;
        }

        path.pop_back();
    }

    return next_bound;
}

int IDAstar_aux::path_contains(std::vector<StateID> &path, StateID state) const {
    for (size_t i = 0; i < path.size(); ++i) {
        if (path[i] == state)
            return i;
    }
    return -1;
}

void IDAstar_aux::dump_search_space() const {
    search_space.dump(task_proxy);
}

void IDAstar_aux::start_f_value_statistics(EvaluationContext &eval_context) {
    int h_value = eval_context.get_evaluator_value(evaluator.get());
    statistics.report_h_value_progress(h_value);
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void IDAstar_aux::update_f_value_statistics(EvaluationContext &eval_context) {
    int h_value = eval_context.get_evaluator_value(evaluator.get());
    statistics.report_h_value_progress(h_value);
}

void add_options_to_feature(plugins::Feature &feature) {
    SearchAlgorithm::add_pruning_option(feature);
    SearchAlgorithm::add_options_to_feature(feature);
}
}
