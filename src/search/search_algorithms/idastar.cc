#include "idastar.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../open_list_factory.h"

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
      evaluator(opts.get<shared_ptr<Evaluator>>("eval", nullptr)),
      opts(opts) {
}

void IDAstar::initialize() {
    State initial_state = state_registry.get_initial_state();
    path.push_back(initial_state.get_id());

    EvaluationContext eval_context(initial_state, 0, true, &statistics);

    search_bound = eval_context.get_evaluator_value_or_infinity(evaluator.get());
}

void IDAstar::print_statistics() const {
    return;
}

SearchStatus IDAstar::step() {
    Plan plan;

    idastar_aux::IDAstar_aux idastar_aux = idastar_aux::IDAstar_aux(opts);
    idastar_aux.initialize();
    
    log << "The current bound is " << search_bound << endl;
    int t = idastar_aux.search(path, search_bound, plan);
    if (t == AUX_SOLVED) {
        set_plan(plan);
        return SOLVED;
    } else if (t == AUX_FAILED || t == numeric_limits<int>::max()) {
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
    int f_value = eval_context.get_evaluator_value_or_infinity(evaluator.get());
    statistics.report_h_value_progress(f_value);
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void IDAstar::update_f_value_statistics(EvaluationContext &eval_context) {
    int f_value = eval_context.get_evaluator_value_or_infinity(evaluator.get());
    statistics.report_h_value_progress(f_value);
}

void add_options_to_feature(plugins::Feature &feature) {
    SearchAlgorithm::add_options_to_feature(feature);
}
}
