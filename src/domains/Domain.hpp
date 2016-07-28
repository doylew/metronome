#ifndef METRONOME_DOMAIN_HPP
#define METRONOME_DOMAIN_HPP

#include <experiment/Configuration.hpp>
#include <limits>
#include <string>
#include <vector>
#include "SuccessorBundle.hpp"

namespace metronome {

class Domain {
public:
    class Action {
        std::string toString() const {
            // TODO
        }
    };
    class State {
        bool operator==(const State& state) const {
            // TODO
        }

        std::size_t hash() const {
            // TODO
        }

        std::string toString() const {
            // TODO
        }
    };

    Domain(const Configuration& configuration, std::istream& input) {
        // TODO
    }

    const State transition(const State& state, const Action& action) const {
        // TODO
    }

    bool isGoal(const State& location) const {
        // TODO
    }

    Cost distance(const State& state) const {
        // TODO
    }

    Cost heuristic(const State& state) const {
        // TODO
    }

    std::vector<SuccessorBundle<Domain>> successors(State state) const {
        // TODO
    }

    const State getStartState() const {
        // TODO
    }
};
}
#endif // METRONOME_DOMAIN_HPP