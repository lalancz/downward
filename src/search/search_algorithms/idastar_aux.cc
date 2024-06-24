#include "idastar.h"
#include "idastar_aux.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../open_list_factory.h"

#include "../algorithms/ordered_set.h"
#include "../plugins/options.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
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
        f_evaluator(opts.get<shared_ptr<Evaluator>>("f_eval", nullptr)) {
}

void IDAstar_aux::initialize() {
    log << "Conducting IDA* aux search" << endl;

    State initial_state = state_registry.get_initial_state();
    SearchNode node = search_space.get_node(initial_state);
    node.open_initial();
}

void IDAstar_aux::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
}

SearchStatus IDAstar_aux::step() {
    return SOLVED;
}

int IDAstar_aux::search(std::vector<StateID> &path, int bound, Plan &plan, SearchStatistics &idastar_statistics) {
    optional<SearchNode> node;

    StateID id = path.back();
    State state = state_registry.lookup_state(id);
    node.emplace(search_space.get_node(state));
    const State &s = node->get_state();

    EvaluationContext eval_context(s, node->get_g(), false, &idastar_statistics);

    statistics.inc_evaluated_states();

    int f = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());

    if (f > bound)
        return f;

    if (task_properties::is_goal_state(task_proxy, state)){
        search_space.trace_path(state, plan);
        return idastar::AUX_SOLVED;
    }

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(s, applicable_ops);

    int next_bound = numeric_limits<int>::max();
    for (OperatorID op_id : applicable_ops) {
        
        OperatorProxy op = task_proxy.get_operators()[op_id];

        State succ_state = state_registry.get_successor_state(s, op);
        idastar_statistics.inc_generated();


        if (path_contains(path, succ_state.get_id()))
            continue; 
            
        SearchNode succ_node = search_space.get_node(succ_state);
        int succ_g = node->get_g() + get_adjusted_cost(op);
        EvaluationContext succ_eval_context(succ_state, succ_g, true, &idastar_statistics);
        idastar_statistics.inc_evaluated_states();

        update_f_value_statistics(succ_eval_context, idastar_statistics);

        if (search_progress.check_progress(succ_eval_context)) {
            idastar_statistics.print_checkpoint_line(succ_g);
        }

        succ_node.open(*node, op, get_adjusted_cost(op));
        idastar_statistics.inc_expanded();

        path.push_back(succ_state.get_id());

        int t = search(path, bound, plan, idastar_statistics);
        if (t == idastar::AUX_SOLVED) {
            return idastar::AUX_SOLVED;
        } else if (t < next_bound) {
            next_bound = t;
        }

        statistics.inc_expanded();
        path.pop_back();
    }

    return next_bound;
}

bool IDAstar_aux::path_contains(std::vector<StateID> &path, StateID state) const {
    for (size_t i = 0; i < path.size(); ++i) {
        if (path[i] == state)
            return true;
    }
    return false;
}

void IDAstar_aux::start_f_value_statistics(EvaluationContext &eval_context) {
    int f_value = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());
    statistics.report_f_value_progress(f_value);
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void IDAstar_aux::update_f_value_statistics(EvaluationContext &eval_context, SearchStatistics &idastar_statistics) {
    int f_value = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());
    idastar_statistics.report_f_value_progress(f_value);
}

void add_options_to_feature(plugins::Feature &feature) {
    SearchAlgorithm::add_options_to_feature(feature);
}
}
