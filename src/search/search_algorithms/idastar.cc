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

const int DELIMITER = 12345;

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
    
    search_bound = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());

    print_initial_evaluator_values(eval_context);
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

    iteration_budgets.push_back(search_bound);

    log << "The current bound is " << search_bound << endl;
    int t = search(operatorPath, solutionPath, 0, task_proxy.get_initial_state(), search_bound, statistics);
    if (t == AUX_SOLVED) {
        iteration_times.push_back(iteration_timer.stop());
        nodes_expanded_per_iteration.push_back(nodes);

        log << "Number of iterations: " << num_of_iterations << endl;

        double average_iteration_time = accumulate(iteration_times.begin(), iteration_times.end(), 0.0) / iteration_times.size();
        log << "Average iteration time: " << average_iteration_time << endl;

        log << "Iteration budgets: ";
        for (int budget : iteration_budgets) {
            log << budget << DELIMITER;
        }
        log << endl;

        log << "Nodes expanded per iteration: ";
        for (int nodes : nodes_expanded_per_iteration) {
            log << nodes << DELIMITER;
        }
        log << endl;

        set_plan(operatorPath);
        return SOLVED;
    } else if (t == numeric_limits<int>::max()) {
        return FAILED;
    }

    search_bound = t;

    iteration_times.push_back(iteration_timer.stop());
    nodes_expanded_per_iteration.push_back(nodes);

    return IN_PROGRESS;
}

int IDAstar::search(std::vector<OperatorID> &operatorPath, std::vector<State> &solutionPath, int pathCost, 
        State currState, int bound, SearchStatistics &idastar_statistics) {

    EvaluationContext eval_context(currState, pathCost, false, &idastar_statistics);
    

    statistics.inc_evaluated_states();

    int f = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());

    if (f > bound)
        return f;

    if (task_properties::is_goal_state(task_proxy, currState)){
        return AUX_SOLVED;
    }

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(currState, applicable_ops);

    nodes++;

    int next_bound = numeric_limits<int>::max();
    for (OperatorID op_id : applicable_ops) {
        OperatorProxy op = task_proxy.get_operators()[op_id];
        State succ_state = currState.get_unregistered_successor(op);
        StateID succ_id = succ_state.get_id();
        idastar_statistics.inc_generated();

        if (pathContains(solutionPath, succ_state))
            continue;
            
        solutionPath.push_back(succ_state);
        operatorPath.push_back(op_id);

        int t = search(operatorPath, solutionPath, pathCost + get_adjusted_cost(op), succ_state, bound, idastar_statistics);
        if (t == AUX_SOLVED) {
            return AUX_SOLVED;
        } else if (t < next_bound) {
            next_bound = t;
        }

        statistics.inc_expanded();

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
