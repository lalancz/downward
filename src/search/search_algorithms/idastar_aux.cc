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

using namespace std;

namespace idastar_aux {
IDAstar_aux::IDAstar_aux(const plugins::Options &opts)
    : SearchAlgorithm(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      open_list(opts.get<shared_ptr<OpenListFactory>>("open")->
                create_state_open_list()),
      f_evaluator(opts.get<shared_ptr<Evaluator>>("f_eval", nullptr)),
      preferred_operator_evaluators(opts.get_list<shared_ptr<Evaluator>>("preferred")),
      lazy_evaluator(opts.get<shared_ptr<Evaluator>>("lazy_evaluator", nullptr)),
      pruning_method(opts.get<shared_ptr<PruningMethod>>("pruning")) {
    if (lazy_evaluator && !lazy_evaluator->does_cache_estimates()) {
        cerr << "lazy_evaluator must cache its estimates" << endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }
}

void IDAstar_aux::initialize() {
    log << "Conducting best first search"
        << (reopen_closed_nodes ? " with" : " without")
        << " reopening closed nodes, (real) bound = " << bound
        << endl;
    assert(open_list);

    set<Evaluator *> evals;
    open_list->get_path_dependent_evaluators(evals);

    /*
      Collect path-dependent evaluators that are used for preferred operators
      (in case they are not also used in the open list).
    */
    for (const shared_ptr<Evaluator> &evaluator : preferred_operator_evaluators) {
        evaluator->get_path_dependent_evaluators(evals);
    }

    /*
      Collect path-dependent evaluators that are used in the f_evaluator.
      They are usually also used in the open list and will hence already be
      included, but we want to be sure.
    */
    if (f_evaluator) {
        f_evaluator->get_path_dependent_evaluators(evals);
    }

    /*
      Collect path-dependent evaluators that are used in the lazy_evaluator
      (in case they are not already included).
    */
    if (lazy_evaluator) {
        lazy_evaluator->get_path_dependent_evaluators(evals);
    }

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
    }

    print_initial_evaluator_values(eval_context);

    pruning_method->initialize(task);
}

void IDAstar_aux::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
    pruning_method->print_statistics();
}

SearchStatus IDAstar_aux::step() {
    return SOLVED;
}

int IDAstar_aux::search(EvaluationContext &eval_context, int g, int bound) {
        if (open_list->empty()) {
        log << "Open list is empty -- no solution!" << endl;
        return FAILED;
    }


    optional<SearchNode> node;

    StateID id = open_list->remove_min();
    State state = state_registry.lookup_state(id);
    node.emplace(search_space.get_node(state));
    const State &s = node->get_state();
    if (check_goal_and_set_plan(s)){
        std::cout << "Found a solution with cost " << node->get_g() << endl;
    
        return SOLVED;
    }
    

    int h = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());
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

        SearchNode succ_node = search_space.get_node(succ_state);
        int succ_g = node->get_g() + get_adjusted_cost(op);
        EvaluationContext succ_eval_context(succ_state, succ_g, false, &statistics);
        statistics.inc_evaluated_states();

        succ_node.open(*node, op, get_adjusted_cost(op));
        open_list->insert(succ_eval_context, succ_state.get_id());

        int succ_h = succ_eval_context.get_evaluator_value_or_infinity(f_evaluator.get());
        int succ_f = succ_g + succ_h;
        if (succ_f > bound) {
            next_bound = min(next_bound, succ_f);
        } else {
            int status = search(succ_eval_context, succ_g, bound);
            if (status != FAILED)
                return status;
        }
    }

    return next_bound;
}

void IDAstar_aux::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
    open_list->boost_preferred();
}

void IDAstar_aux::dump_search_space() const {
    search_space.dump(task_proxy);
}

void IDAstar_aux::start_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_evaluator_value(f_evaluator.get());
        statistics.report_f_value_progress(f_value);
    }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void IDAstar_aux::update_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_evaluator_value(f_evaluator.get());
        statistics.report_f_value_progress(f_value);
    }
}

void add_options_to_feature(plugins::Feature &feature) {
    SearchAlgorithm::add_pruning_option(feature);
    SearchAlgorithm::add_options_to_feature(feature);
}
}
