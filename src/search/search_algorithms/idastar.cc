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
      opts(opts),
      f_evaluator(opts.get<shared_ptr<Evaluator>>("f_eval", nullptr)) {
}

void IDAstar::initialize() {
    log << "Conducting IDA* search" << endl;

    State initial_state = state_registry.get_initial_state();
    path.push_back(initial_state.get_id());

    EvaluationContext eval_context(initial_state, 0, true, &statistics);

    SearchNode node = search_space.get_node(initial_state);
    node.open_initial();

    if (search_progress.check_progress(eval_context))
        statistics.print_checkpoint_line(0);
    start_f_value_statistics(eval_context);

    search_bound = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());

    print_initial_evaluator_values(eval_context);
}

void IDAstar::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
}

SearchStatus IDAstar::step() {
    Plan plan;

    idastar_aux::IDAstar_aux idastar_aux = idastar_aux::IDAstar_aux(opts);
    idastar_aux.initialize();
    
    log << "The current bound is " << search_bound << endl;
    int t = idastar_aux.search(path, search_bound, plan, statistics);
    if (t == AUX_SOLVED) {
        set_plan(plan);
        return SOLVED;
    } else if (t == AUX_FAILED || t == numeric_limits<int>::max()) {
        return FAILED;
    }

    search_bound = t;

    return IN_PROGRESS;
}

void IDAstar::start_f_value_statistics(EvaluationContext &eval_context) {
    int f_value = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());
    statistics.report_f_value_progress(f_value);
}

void add_options_to_feature(plugins::Feature &feature) {
    SearchAlgorithm::add_options_to_feature(feature);
}
}
