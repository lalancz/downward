#include "landmark_factory_relaxation.h"

#include "exploration.h"
#include "landmark.h"

#include "../task_utils/task_properties.h"

using namespace std;

namespace landmarks {
void LandmarkFactoryRelaxation::generate_landmarks(const shared_ptr<AbstractTask> &task) {
    TaskProxy task_proxy(*task);
    Exploration exploration(task_proxy);
    generate_relaxed_landmarks(task, exploration);
    postprocess(task_proxy, exploration);
}

void LandmarkFactoryRelaxation::postprocess(
    const TaskProxy &task_proxy, Exploration &exploration) {
    lm_graph->set_landmark_ids();
    calc_achievers(task_proxy, exploration);
    mk_acyclic_graph();
}

void LandmarkFactoryRelaxation::discard_noncausal_landmarks(
    const TaskProxy &task_proxy, Exploration &exploration) {
    // TODO: Check if the code works correctly in the presence of axioms.
    task_properties::verify_no_conditional_effects(task_proxy);
    int num_all_landmarks = lm_graph->get_num_landmarks();
    lm_graph->remove_node_if(
        [this, &task_proxy, &exploration](const LandmarkNode &node) {
            return !is_causal_landmark(task_proxy, exploration, node.get_landmark());
        });
    int num_causal_landmarks = lm_graph->get_num_landmarks();
    utils::g_log << "Discarded " << num_all_landmarks - num_causal_landmarks
                 << " non-causal landmarks" << endl;
}

bool LandmarkFactoryRelaxation::is_causal_landmark(
    const TaskProxy &task_proxy, Exploration &exploration,
    const Landmark &landmark) const {
    assert(!landmark.conjunctive);

    if (landmark.is_true_in_goal)
        return true;

    unordered_set<int> excluded_op_ids;
    vector<FactPair> excluded_props;
    for (OperatorProxy op : task_proxy.get_operators()) {
        if (is_landmark_precondition(op, landmark)) {
            excluded_op_ids.insert(op.get_id());
        }
    }

    // TODO: try to do this over relaxed_task_solvable
    vector<vector<bool>> reached =
        exploration.compute_relaxed_reachability(excluded_props,
                                                 excluded_op_ids);

    for (FactProxy goal : task_proxy.get_goals()) {
        if (!reached[goal.get_variable().get_id()][goal.get_value()]) {
            return true;
        }
    }
    return false;
}

void LandmarkFactoryRelaxation::calc_achievers(
    const TaskProxy &task_proxy, Exploration &exploration) {
    assert(!achievers_calculated);
    VariablesProxy variables = task_proxy.get_variables();
    for (auto &lm_node : lm_graph->get_nodes()) {
        Landmark &landmark = lm_node->get_landmark();
        for (const FactPair &lm_fact : landmark.facts) {
            const vector<int> &ops = get_operators_including_eff(lm_fact);
            landmark.possible_achievers.insert(ops.begin(), ops.end());

            if (variables[lm_fact.var].is_derived())
                landmark.is_derived = true;
        }

        vector<vector<bool>> reached =
            relaxed_reachability(task_proxy, exploration, landmark);

        for (int op_or_axom_id : landmark.possible_achievers) {
            OperatorProxy op = get_operator_or_axiom(task_proxy, op_or_axom_id);

            if (possibly_reaches_lm(op, reached, landmark)) {
                landmark.first_achievers.insert(op_or_axom_id);
            }
        }
    }
    achievers_calculated = true;
}

bool LandmarkFactoryRelaxation::relaxed_task_solvable(
    const TaskProxy &task_proxy, Exploration &exploration,
    const Landmark &exclude) const {
    vector<vector<bool>> reached =
        relaxed_reachability(task_proxy, exploration, exclude);

    for (FactProxy goal : task_proxy.get_goals()) {
        if (!reached[goal.get_variable().get_id()][goal.get_value()]) {
            return false;
        }
    }
    return true;
}

vector<vector<bool>> LandmarkFactoryRelaxation::relaxed_reachability(
    const TaskProxy &task_proxy, Exploration &exploration,
        const Landmark &exclude) const {

    // Extract propositions from "exclude"
    unordered_set<int> excluded_op_ids;
    vector<FactPair> excluded_props;
    for (OperatorProxy op : task_proxy.get_operators()) {
        if (achieves_non_conditional(op, exclude))
            excluded_op_ids.insert(op.get_id());
    }
    excluded_props.insert(excluded_props.end(),
                         exclude.facts.begin(), exclude.facts.end());

    return exploration.compute_relaxed_reachability(excluded_props,
                                                    excluded_op_ids);
}

bool LandmarkFactoryRelaxation::achieves_non_conditional(
    const OperatorProxy &o, const Landmark &landmark) const {
    /* Test whether the landmark is achieved by the operator unconditionally.
    A disjunctive landmark is achieved if one of its disjuncts is achieved. */
    for (EffectProxy effect: o.get_effects()) {
        for (const FactPair &lm_fact : landmark.facts) {
            FactProxy effect_fact = effect.get_fact();
            if (effect_fact.get_pair() == lm_fact) {
                if (effect.get_conditions().empty())
                    return true;
            }
        }
    }
    return false;
}
}
