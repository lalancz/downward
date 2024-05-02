#include "idastar.h"

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

namespace idastar {
IDAstar::IDAstar(const plugins::Options &opts)
    : SearchAlgorithm(opts),
      open_list(opts.get<shared_ptr<OpenListFactory>>("open")->
                create_state_open_list()),
      evaluator(opts.get<shared_ptr<Evaluator>>("eval", nullptr)),
      idastar_aux(opts) {
}

void IDAstar::initialize() {
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


    search_bound = eval_context.get_evaluator_value(evaluator.get());
}

void IDAstar::print_statistics() const {
    idastar_aux.print_statistics();
}

SearchStatus IDAstar::step() {
    if (open_list->empty()) {
        log << "Open list is empty -- no solution!" << endl;
        return FAILED;
    }

    log << "Bound is " << search_bound << endl;
    int t = idastar_aux.search(path, search_bound, open_list.get());
    if (t == SOLVED) {
        check_goal_and_set_plan(state_registry.lookup_state(idastar_aux.path.back()));
        return SOLVED;
    } else if (t == FAILED || t == numeric_limits<int>::max()) {
        return FAILED;
    }

    search_bound = t;

    return IN_PROGRESS;
}

void IDAstar::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
    open_list->boost_preferred();
}

void IDAstar::dump_search_space() const {
    search_space.dump(task_proxy);
}

void IDAstar::start_f_value_statistics(EvaluationContext &eval_context) {
    int f_value = eval_context.get_evaluator_value(evaluator.get());
    statistics.report_h_value_progress(f_value);
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void IDAstar::update_f_value_statistics(EvaluationContext &eval_context) {
    int f_value = eval_context.get_evaluator_value(evaluator.get());
    statistics.report_h_value_progress(f_value);
}

void add_options_to_feature(plugins::Feature &feature) {
    SearchAlgorithm::add_pruning_option(feature);
    SearchAlgorithm::add_options_to_feature(feature);
}
}
