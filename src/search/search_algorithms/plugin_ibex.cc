#include "ibex.h"
#include "search_common.h"

#include "../plugins/plugin.h"

using namespace std;

namespace plugin_ibex {
class IBEXFeature : public plugins::TypedFeature<SearchAlgorithm, ibex::IBEX> {
public:
    IBEXFeature() : TypedFeature("ibex") {
        document_title("IBEX search");
        document_synopsis("");

        add_option<shared_ptr<Evaluator>>("eval", "evaluator");

        add_option<int>(
            "c_1",
            "c_1",
            "2");

        add_option<int>(
            "c_2",
            "c_2",
            "8");

        ibex::add_options_to_feature(*this);
    }

    virtual shared_ptr<ibex::IBEX> create_component(const plugins::Options &options, const utils::Context &) const override {
        plugins::Options options_copy(options);
        auto temp = search_common::create_astar_open_list_factory_and_f_eval(options);
        options_copy.set("open", temp.first);
        options_copy.set("f_eval", temp.second);
        return make_shared<ibex::IBEX>(options_copy);
    }
};

static plugins::FeaturePlugin<IBEXFeature> _plugin;
}
