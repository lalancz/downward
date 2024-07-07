#include "ibex.h"

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
#include <math.h>
#include <vector>
#include <numeric>

using namespace std;

const int DELIMITER = 12345;

namespace ibex {
IBEX::IBEX(const plugins::Options &opts)
    : SearchAlgorithm(opts),
      opts(opts),
      evaluator(opts.get<shared_ptr<Evaluator>>("eval", nullptr)),
      c_1(opts.get<int>("c_1")),
      c_2(opts.get<int>("c_2")),
      force_idastar(opts.get<bool>("force_idastar")) {
}

void IBEX::initialize() {
    // force nodes >= c_1 * budget to trigger in step function
    if (force_idastar)
        nodes = 1000000;

    num_of_iterations = 0;

    State initial_state = task_proxy.get_initial_state();

    EvaluationContext eval_context(initial_state, 0, false, &statistics);
    statistics.inc_evaluated_states();

    solutionCost = numeric_limits<int>::max();
    budget = 0;
    i = make_pair(eval_context.get_evaluator_value_or_infinity(evaluator.get()), numeric_limits<int>::max());
}

void IBEX::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
}

std::pair<int, int> IBEX::interval_intersection(std::pair<int, int> i1, std::pair<int, int> i2) {
    if (i1.first > i2.second || i2.first > i1.second) {
        return make_pair(0, 0);
    }

    return make_pair(max(i1.first, i2.first), min(i1.second, i2.second));
}

SearchStatus IBEX::step() {
    while (solutionCost > i.first) {
        log << "i = [" << i.first << ", " << i.second << "]" << endl;
        num_of_iterations++;

        solutionLowerBound = i.first;
        i.second = numeric_limits<int>::max();

        i = interval_intersection(i, search(i.first, numeric_limits<int>::max()));
        log << "i = [" << i.first << ", " << i.second << "]" << endl;
        if (nodes >= c_1 * budget) {
            budget = nodes;
            continue;
        }

        log << "Past the first search" << endl;

        int delta = 0;
        int nextCost;
        while ((i.second != i.first) && (nodes < c_1 * budget)) {
            nextCost = static_cast<int>(i.first + pow(2, delta));
            delta++;
            solutionLowerBound = i.first;
            i = interval_intersection(i, search(nextCost, c_2 * budget));
            log << "i = [" << i.first << ", " << i.second << "]" << endl;
        }

        while ((i.second != i.first) && !((c_1 * budget <= nodes) && (nodes < c_2 * budget))) {
            nextCost = (i.first + i.second) / 2;
            solutionLowerBound = i.first;
            i = interval_intersection(i, search(nextCost, c_2 * budget));
            log << "i = [" << i.first << ", " << i.second << "]" << endl;
        }

        budget = max(nodes, c_1 * budget);

        if (check_goal()) 
            return SOLVED;
    }

    if (check_goal()) 
        return SOLVED;
    
    return FAILED;
}

std::pair<int, int> IBEX::search(int costLimit, int nodeLimit) {
    goalFoundCurrentIteration = false;
    f_below = 0;
    f_above = numeric_limits<int>::max();
    nodes = 0;

    State initial_state = task_proxy.get_initial_state();

    std::vector<State> currentPath;
    std::vector<OperatorID> solutionPath;

    utils::Timer iteration_timer;
    
    limitedDFS(initial_state, 0, costLimit, nodeLimit, currentPath, solutionPath);

    iteration_times.push_back(iteration_timer.stop());
    iteration_budgets.push_back(costLimit);
    nodes_expanded_per_iteration.push_back(nodes);


    if (nodes >= nodeLimit) {
        return make_pair(0, f_below);
    } else if (f_below >= solutionCost) {
        return make_pair(solutionCost, solutionCost);
    } else {
        return make_pair(f_above, numeric_limits<int>::max());
    }
}

void IBEX::limitedDFS(State currState, int pathCost, int costLimit, int nodeLimit, vector<State> &currentPath,
        vector<OperatorID> &currentSolutionPath) {

    EvaluationContext eval_context(currState, pathCost, false, &statistics);
    statistics.inc_evaluated_states();

    int value = eval_context.get_evaluator_value_or_infinity(evaluator.get());

    int currF;
    if (value == EvaluationResult::INFTY) {
        currF = value;
    } else {
        currF = pathCost + value;
    }

    if (solutionCost == solutionLowerBound) {
        return;
    } else if (currF > costLimit) {
        f_above = min(f_above, currF);
        return;
    } else if (currF >= solutionCost) {
        f_below = solutionCost;
        return;
    } else {
        f_below = max(currF, f_below);
    }

    if (nodes >= nodeLimit) {
        return;
    }

    if (task_properties::is_goal_state(task_proxy, currState)) {
        goalFoundCurrentIteration = true;
        solutionPath = currentSolutionPath;
        solutionCost = currF;
        log << "Goal found with cost: " << solutionCost << endl;
        return;
    }

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(currState, applicable_ops);
    statistics.inc_expanded();

    nodes++;

    for (OperatorID op_id : applicable_ops) {
        OperatorProxy op = task_proxy.get_operators()[op_id];
        State succ_state = currState.get_unregistered_successor(op);
        statistics.inc_generated();

        int succ_g = pathCost + get_adjusted_cost(op);
        
        currentSolutionPath.push_back(op_id);
        currentPath.push_back(succ_state);

        limitedDFS(succ_state, succ_g, costLimit, nodeLimit, currentPath, currentSolutionPath);

        currentPath.pop_back();
        currentSolutionPath.pop_back();
    }
}

bool IBEX::check_goal() {
    if ((solutionCost == i.first) & !(solutionPath.empty())) {
        log << "Solution found with cost: " << solutionCost << endl;
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

        set_plan(solutionPath);
        return true;
    }

    return false;
}

void add_options_to_feature(plugins::Feature &feature) {
    SearchAlgorithm::add_options_to_feature(feature);
}
}
