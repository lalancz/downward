#include "ibex.h"

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
#include <math.h>
#include <vector>

using namespace std;

namespace ibex {
IBEX::IBEX(const plugins::Options &opts)
    : SearchAlgorithm(opts),
      opts(opts),
      evaluator(opts.get<shared_ptr<Evaluator>>("eval", nullptr)),
      c_1(opts.get<int>("c_1")),
      c_2(opts.get<int>("c_2")) {
}

void IBEX::initialize() {
    State initial_state = task_proxy.get_initial_state();

    EvaluationContext eval_context(initial_state, 0, false, &statistics);

    start_f_value_statistics(eval_context);
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

        solutionLowerBound = i.first;
        i.second = numeric_limits<int>::max();

        i = interval_intersection(i, search(i.first, numeric_limits<int>::max()));
        if (nodes >= c_1 * budget) {
            budget = nodes;
            continue;
        }

        int delta = 0;
        int nextCost;
        while ((i.second != i.first) && (nodes < c_1 * budget)) {
            nextCost = static_cast<int>(i.first + pow(2, delta));
            delta++;
            solutionLowerBound = i.first;
            i = interval_intersection(i, search(nextCost, c_2 * budget));
        }

        while ((i.second != i.first) && !((c_1 * budget <= nodes) && (nodes < c_2 * budget))) {
            nextCost = (i.first + i.second) / 2;
            solutionLowerBound = i.first;
            i = interval_intersection(i, search(nextCost, c_2 * budget));
        }

        budget = max(nodes, c_1 * budget);

        if ((solutionCost == i.first) & !(solutionPath.empty())) {
            log << "Solution found with cost " << solutionCost << endl;
            set_plan(solutionPath);
            return SOLVED;
        }
    }

    if ((solutionCost == i.first) & !(solutionPath.empty())) {
        log << "Solution found with cost (outside while loop) " << solutionCost << endl;
        set_plan(solutionPath);
        return SOLVED;
    }
    
    return FAILED;
}

std::pair<int, int> IBEX::search(int costLimit, int nodeLimit) {
    goalFoundCurrentIteration = false;
    f_below = 0;
    f_above = numeric_limits<int>::max();
    nodes = 0;

    vector<OperatorID> currentSolutionPath;

    State initial_state = task_proxy.get_initial_state();

    limitedDFS(initial_state, 0, costLimit, nodeLimit, currentSolutionPath);

    if (nodes >= nodeLimit) {
        return make_pair(0, f_below);
    } else if (f_below >= solutionCost) {
        return make_pair(solutionCost, solutionCost);
    } else {
        return make_pair(f_above, numeric_limits<int>::max());
    }
}

void IBEX::limitedDFS(State currState, int pathCost, int costLimit, int nodeLimit, vector<OperatorID> &currentSolutionPath) {
    EvaluationContext eval_context(currState, pathCost, false, &statistics);
    statistics.inc_evaluated_states();
    update_f_value_statistics(eval_context);

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

    nodes++;

    for (OperatorID op_id : applicable_ops) {
        OperatorProxy op = task_proxy.get_operators()[op_id];
        State succ_state = currState.get_unregistered_successor(op);
        statistics.inc_generated();

        int succ_g = pathCost + get_adjusted_cost(op);

        currentSolutionPath.push_back(op_id);

        limitedDFS(succ_state, succ_g, costLimit, nodeLimit, currentSolutionPath);

        if (goalFoundCurrentIteration) {
            return;
        }

        currentSolutionPath.pop_back();
    }

    statistics.inc_expanded();
}

void IBEX::start_f_value_statistics(EvaluationContext &eval_context) {
    int f_value = eval_context.get_evaluator_value_or_infinity(evaluator.get());
    statistics.report_f_value_progress(f_value);
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void IBEX::update_f_value_statistics(EvaluationContext &eval_context) {
    int f_value = eval_context.get_evaluator_value_or_infinity(evaluator.get());
    statistics.report_f_value_progress(f_value);
}

void add_options_to_feature(plugins::Feature &feature) {
    SearchAlgorithm::add_options_to_feature(feature);
}
}
