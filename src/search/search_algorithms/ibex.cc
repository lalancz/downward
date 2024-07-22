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

namespace ibex {
IBEX::IBEX(const plugins::Options &opts)
    : SearchAlgorithm(opts),
      opts(opts),
      evaluator(opts.get<shared_ptr<Evaluator>>("eval", nullptr)),
      c_1(opts.get<int>("c_1")),
      c_2(opts.get<int>("c_2")),
      force_idastar(opts.get<bool>("force_idastar")),
      path_checking(opts.get<bool>("path_checking")) {
}

void IBEX::initialize() {
    // force nodes >= c_1 * budget to trigger in step function
    log << "Conducting IBEX search" << endl;

    exp_search_triggered = 0;

    num_of_iterations = 0;

    State initial_state = task_proxy.get_initial_state();

    EvaluationContext eval_context(initial_state, 0, false, &statistics);
    statistics.inc_evaluated_states();

    solutionCost = numeric_limits<int>::max();
    budget = 0;
    i = make_pair(eval_context.get_evaluator_value_or_infinity(evaluator.get()), numeric_limits<int>::max());

    log << "i = [" << i.first << ", " << i.second << "]" << endl;
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
        num_of_iterations++;

        solutionLowerBound = i.first;
        i.second = numeric_limits<int>::max();
        log << "i = [" << i.first << ", " << i.second << "]" << endl;

        i = interval_intersection(i, search(i.first, numeric_limits<int>::max()));
        log << "i = [" << i.first << ", " << i.second << "]" << endl;
        log << "Nodes expanded in current regular IDA* iteration: " << nodes << endl;
        if (nodes >= c_1 * budget || force_idastar) {
            budget = nodes;
            log << "Did exp search trigger: 0" << endl;
            continue;
        }

        log << "Did exp search trigger: 1" << endl;

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
    f_below = 0;
    f_above = numeric_limits<int>::max();
    nodes = 0;
    
    currentPath.clear();
    solutionPathOps.clear();

    utils::Timer iteration_timer;
    
    limitedDFS(task_proxy.get_initial_state(), 0, costLimit, nodeLimit);

    log << "Iteration took (seconds): " << iteration_timer.stop() << endl;

    log << "Iteration bound: " << costLimit << endl;

    log << "Nodes expanded in current iteration: " << nodes << endl;

    if (nodes >= nodeLimit) {
        return make_pair(0, f_below);
    } else if (f_below >= solutionCost) {
        return make_pair(solutionCost, solutionCost);
    } else {
        return make_pair(f_above, numeric_limits<int>::max());
    }
}

void IBEX::limitedDFS(State currState, int pathCost, int costLimit, int nodeLimit) {

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
        solutionPath = solutionPathOps;
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

        if (path_checking && pathContains(currentPath, succ_state))
            continue;

        int succ_g = pathCost + get_adjusted_cost(op);
        
        solutionPathOps.push_back(op_id);

        if (path_checking)
            currentPath.push_back(succ_state);

        limitedDFS(succ_state, succ_g, costLimit, nodeLimit);

        solutionPathOps.pop_back();

        if (path_checking)
            currentPath.pop_back();
    }
}

bool IBEX::pathContains(std::vector<State> &path, State state) {
    for (State state_temp : path) {
        if (state_temp == state) {
            return true;
        }
    }
    return false;
}

bool IBEX::check_goal() {
    if ((solutionCost == i.first) & !(solutionPath.empty())) {
        log << "Solution found with cost: " << solutionCost << endl;
        
        log << "Number of iterations: " << num_of_iterations << endl;

        set_plan(solutionPath);
        return true;
    }

    return false;
}

void add_options_to_feature(plugins::Feature &feature) {
    SearchAlgorithm::add_options_to_feature(feature);
}
}
