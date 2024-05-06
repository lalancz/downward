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
        open_list(opts.get<shared_ptr<OpenListFactory>>("open")->create_state_open_list()),
        evaluator(opts.get<shared_ptr<Evaluator>>("eval", nullptr)) {
}

void IDAstar_aux::initialize() {
    log << "Conducting IDA* search" << endl;
    assert(open_list);

    set<Evaluator *> evals;
    open_list->get_path_dependent_evaluators(evals);

    path_dependent_evaluators.assign(evals.begin(), evals.end());

    State initial_state = state_registry.get_initial_state();
    for (Evaluator *evaluator : path_dependent_evaluators) {
        evaluator->notify_initial_state(initial_state);
    }

    /*
      Note: we consider the initial state as reached by a preferred
      operator.
    */
    EvaluationContext eval_context(initial_state, 0, true, &statistics);

    statistics.inc_evaluated_states();

    if (open_list->is_dead_end(eval_context)) {
        log << "Initial state is a dead end." << endl;
    } else {
        if (search_progress.check_progress(eval_context))
            statistics.print_checkpoint_line(0);
        start_f_value_statistics(eval_context);
        SearchNode node = search_space.get_node(initial_state);
        node.open_initial();

        open_list->insert(eval_context, initial_state.get_id());
        path.push_back(initial_state.get_id());
    }

    print_initial_evaluator_values(eval_context);
}

void IDAstar_aux::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
}

SearchStatus IDAstar_aux::step() {
    return SOLVED;
}

int IDAstar_aux::search(std::vector<StateID> &path, int bound) {
    optional<SearchNode> node;

    StateID id = path.back();
    State state = state_registry.lookup_state(id);
    node.emplace(search_space.get_node(state));
    const State &s = node->get_state();
    if (task_properties::is_goal_state(task_proxy, state)){    
        return idastar::AUX_SOLVED;
    }
    
    EvaluationContext eval_context(s, node->get_g(), false, &statistics);
    int h = eval_context.get_evaluator_value_or_infinity(evaluator.get());
    int f = h + node->get_g();

    if (f > bound)
        return f;

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(s, applicable_ops);

    int next_bound = numeric_limits<int>::max();
    for (OperatorID op_id : applicable_ops) {
        
        OperatorProxy op = task_proxy.get_operators()[op_id];

        State succ_state = state_registry.get_successor_state(s, op);
        statistics.inc_generated();


        if (path_contains(path, succ_state.get_id()))
            continue; 
            
        SearchNode succ_node = search_space.get_node(succ_state);
        int succ_g = node->get_g() + get_adjusted_cost(op);
        EvaluationContext succ_eval_context(succ_state, succ_g, false, &statistics);
        statistics.inc_evaluated_states();

        succ_node.open(*node, op, get_adjusted_cost(op));
        path.push_back(succ_state.get_id());

        int t = search(path, bound);
        if (t == idastar::AUX_SOLVED) {
            return idastar::AUX_SOLVED;
        } else if (t < next_bound) {
            next_bound = t;
        }

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

void IDAstar_aux::dump_search_space() const {
    search_space.dump(task_proxy);
}

void IDAstar_aux::start_f_value_statistics(EvaluationContext &eval_context) {
    int h_value = eval_context.get_evaluator_value_or_infinity(evaluator.get());
    statistics.report_h_value_progress(h_value);
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void IDAstar_aux::update_f_value_statistics(EvaluationContext &eval_context) {
    int h_value = eval_context.get_evaluator_value_or_infinity(evaluator.get());
    statistics.report_h_value_progress(h_value);
}

void add_options_to_feature(plugins::Feature &feature) {
    SearchAlgorithm::add_options_to_feature(feature);
}
}
