#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#define CATCH_CONFIG_COLOUR_NONE

#include "../dependencies/catch.hpp"
#include <boost/multi_array.hpp>
#include "easylogging++.h"
#include "util/PriorityQueueTest.hpp"
#include "algorithm/AStarTest.hpp"
#include "domains/VacuumWorldTest.hpp"

INITIALIZE_EASYLOGGINGPP

TEST_CASE("Main test", "[main]") {
}
