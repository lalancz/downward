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
};

static plugins::FeaturePlugin<IBEXFeature> _plugin;
}
