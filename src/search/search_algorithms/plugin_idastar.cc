#include "idastar.h"
#include "search_common.h"

#include "../plugins/plugin.h"

using namespace std;

namespace plugin_idastar {
class IDAstarFeature : public plugins::TypedFeature<SearchAlgorithm, idastar::IDAstar> {
public:
    IDAstarFeature() : TypedFeature("idastar") {
        document_title("IDA* search");
        document_synopsis("");

        add_option<shared_ptr<Evaluator>>("eval", "evaluator");

        add_option<bool>(
            "path_checking",
            "path_checking",
            "false");

        idastar::add_options_to_feature(*this);
    }

    virtual shared_ptr<idastar::IDAstar> create_component(const plugins::Options &options, const utils::Context &) const override {
        plugins::Options options_copy(options);
        auto temp = search_common::create_astar_open_list_factory_and_f_eval(options);
        options_copy.set("f_eval", temp.second);
        return make_shared<idastar::IDAstar>(options_copy);
    }
};

static plugins::FeaturePlugin<IDAstarFeature> _plugin;
}
