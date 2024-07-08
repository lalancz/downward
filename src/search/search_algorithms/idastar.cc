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
      f_evaluator(opts.get<shared_ptr<Evaluator>>("f_eval", nullptr)) {
}

void IDAstar::initialize() {
    log << "Conducting IDA* search" << endl;

    num_of_iterations = 0;

    State initial_state = task_proxy.get_initial_state();

    solutionPath.push_back(initial_state);

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

    operatorPath.clear();
    solutionPath.clear();

    operatorPath.reserve(search_bound);
    solutionPath.reserve(search_bound);

    total_iteration_budgets = total_iteration_budgets + search_bound;

    log << "The current bound is " << search_bound << endl;
    int t = search(operatorPath, solutionPath, 0, task_proxy.get_initial_state(), search_bound);
    if (t == AUX_SOLVED) {
        total_iteration_times = total_iteration_times + iteration_timer.stop();
        total_nodes_expanded_per_iteration= total_nodes_expanded_per_iteration + nodes;

        log << "Number of iterations: " << num_of_iterations << endl;

        double average_iteration_time = total_iteration_times / num_of_iterations;
        log << "Average iteration time: " << average_iteration_time << endl;

        float average_budget = total_iteration_budgets / num_of_iterations;
        log << "Average iteration budget: " << average_budget << endl;

        float average_nodes_expanded = total_nodes_expanded_per_iteration / num_of_iterations;
        log << "Average nodes expanded per iteration: " << average_nodes_expanded << endl;

        set_plan(operatorPath);
        return SOLVED;
    } else if (t == numeric_limits<int>::max()) {
        return FAILED;
    }

    search_bound = t;

    total_iteration_times = total_iteration_times + iteration_timer.stop();
    total_nodes_expanded_per_iteration= total_nodes_expanded_per_iteration + nodes;

    return IN_PROGRESS;
}

int IDAstar::search(std::vector<OperatorID> &operatorPath, std::vector<State> &solutionPath, int pathCost, 
        State currState, int bound) {

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

        if (pathContains(solutionPath, succ_state))
            continue;
            
        solutionPath.push_back(succ_state);
        operatorPath.push_back(op_id);

        int t = search(operatorPath, solutionPath, pathCost + get_adjusted_cost(op), succ_state, bound);
        if (t == AUX_SOLVED) {
            return AUX_SOLVED;
        } else if (t < next_bound) {
            next_bound = t;
        }

        solutionPath.pop_back();
        operatorPath.pop_back();
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
