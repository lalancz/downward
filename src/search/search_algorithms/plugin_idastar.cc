#include "idastar.h"
#include "search_common.h"

#include "../plugins/plugin.h"

using namespace std;

namespace plugin_idastar {
class IDAstarFeature : public plugins::TypedFeature<SearchAlgorithm, idastar::IDAstar> {
public:
    IDAstarFeature() : TypedFeature("idastar") {
        document_title("Eager best-first search");
        document_synopsis("");

        add_option<shared_ptr<OpenListFactory>>("open", "open list");
        add_option<bool>(
            "reopen_closed",
            "reopen closed nodes",
            "false");
        add_option<shared_ptr<Evaluator>>(
            "f_eval",
            "set evaluator for jump statistics. "
            "(Optional; if no evaluator is used, jump statistics will not be displayed.)",
            plugins::ArgumentInfo::NO_DEFAULT);
        add_list_option<shared_ptr<Evaluator>>(
            "preferred",
            "use preferred operators of these evaluators",
            "[]");
        idastar::add_options_to_feature(*this);
    }
};

static plugins::FeaturePlugin<IDAstarFeature> _plugin;
}
