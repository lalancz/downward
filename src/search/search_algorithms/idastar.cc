#include "idastar.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../open_list_factory.h"

#include "../algorithms/ordered_set.h"
#include "../plugins/options.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"
#include "../utils/timer.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <optional>
#include <set>
#include <stack>
#include <numeric>

using namespace std;

namespace idastar {
IDAstar::IDAstar(const plugins::Options &opts)
    : SearchAlgorithm(opts),
      opts(opts),
      f_evaluator(opts.get<shared_ptr<Evaluator>>("f_eval", nullptr)),
      path_checking(opts.get<bool>("path_checking")) {
}

void IDAstar::initialize() {
    log << "Conducting IDA* search" << endl;

    num_of_iterations = 0;

    State initial_state = task_proxy.get_initial_state();

    EvaluationContext eval_context(initial_state, 0, true, &statistics);
    statistics.inc_evaluated_states();
    
    search_bound = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());
}

void IDAstar::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
}

SearchStatus IDAstar::step() {
    num_of_iterations++;
    nodes = 0;
    utils::Timer iteration_timer;

    currentPath.clear();
    solutionPathOps.clear();

    log << "Iteration bound: " << search_bound << endl;
    int t = search(task_proxy.get_initial_state(), 0, search_bound);
    if (t == AUX_SOLVED) {
        log << "Number of iterations: " << num_of_iterations << endl;

        log << "Iteration took (seconds): " << iteration_timer.stop() << endl;
    
        log << "Nodes expanded in current iteration: " << nodes << endl;

        set_plan(solutionPathOps);
        return SOLVED;
    } else if (t == numeric_limits<int>::max()) {
        return FAILED;
    }

    log << "Iteration took (seconds): " << iteration_timer.stop() << endl;

    log << "Nodes expanded in current iteration: " << nodes << endl;

    search_bound = t;

    return IN_PROGRESS;
}

int IDAstar::search(State currState, int pathCost, int bound) {

    EvaluationContext eval_context(currState, pathCost, false, &statistics);
    statistics.inc_evaluated_states();

    int f = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());

    if (f > bound)
        return f;

    if (task_properties::is_goal_state(task_proxy, currState)){
        return AUX_SOLVED;
    }

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(currState, applicable_ops);
    statistics.inc_expanded();

    nodes++;

    int next_bound = numeric_limits<int>::max();
    for (OperatorID op_id : applicable_ops) {
        OperatorProxy op = task_proxy.get_operators()[op_id];
        State succ_state = currState.get_unregistered_successor(op);
        statistics.inc_generated();
        StateID succ_id = succ_state.get_id();

        if (path_checking && pathContains(currentPath, succ_state))
            continue;
            
        solutionPathOps.push_back(op_id);

        if (path_checking)
            currentPath.push_back(succ_state);

        int t = search(succ_state, pathCost + get_adjusted_cost(op), bound);
        if (t == AUX_SOLVED) {
            return AUX_SOLVED;
        } else if (t < next_bound) {
            next_bound = t;
        }

        solutionPathOps.pop_back();

        if (path_checking)
            currentPath.pop_back();
    }

    return next_bound;
}

bool IDAstar::pathContains(std::vector<State> &path, State state) {
    for (State state_temp : path) {
        if (state_temp == state) {
            return true;
        }
    }
    return false;
}

void add_options_to_feature(plugins::Feature &feature) {
    SearchAlgorithm::add_options_to_feature(feature);
}
}
