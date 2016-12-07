#ifndef METRONOME_SZEROFIVE_HPP
#define METRONOME_SZEROFIVE_HPP
#include <fcntl.h>
#include <algorithm>
#include <domains/SuccessorBundle.hpp>
#include <unordered_map>
#include <vector>
#include "MemoryConfiguration.hpp"
#include "MetronomeException.hpp"
#include "OnlinePlanner.hpp"
#include "utils/Hasher.hpp"
#include "utils/PriorityQueue.hpp"
#include "utils/StaticVector.hpp"

namespace metronome {

template <typename Domain, typename TerminationChecker>
class SZeroFive final : public OnlinePlanner<Domain, TerminationChecker> {
public:
    typedef typename Domain::State State;
    typedef typename Domain::Action Action;
    typedef typename Domain::Cost Cost;
    typedef typename OnlinePlanner<Domain, TerminationChecker>::ActionBundle ActionBundle;

    SZeroFive(const Domain& domain, const Configuration&) : domain{domain} {
        //        // Force the object pool to allocate memory
        //        State state;
        //        Node node = Node(nullptr, std::move(state), Action(), 0, 0, true);
        //        nodePool.destroy(nodePool.construct(node));

        // Initialize hash table
        nodes.max_load_factor(1);
        nodes.reserve(Memory::NODE_LIMIT);
    }

    std::vector<ActionBundle> selectActions(const State& startState, TerminationChecker& terminationChecker) override {
        if (domain.isGoal(startState)) {
            // Goal is already reached
            return std::vector<ActionBundle>();
        }
        if (openList.isNotEmpty()) {
            learn(terminationChecker);
        }
        const auto bestNode = explore(startState, terminationChecker);
        // Learning phase

        sweepBackSafety();
        return extractPath(bestNode, nodes[startState]);
    }

private:
    class Edge;

    class Node {
    public:
        Node(Node* parent, const State& state, Action action, Cost g, Cost h, bool open, unsigned int iteration = 0)
                : parent{parent},
                  state{state},
                  action{std::move(action)},
                  g{g},
                  h{h},
                  open{open},
                  iteration{iteration} {}

        Cost f() const { return g + h; }

        unsigned long hash() const { return state.hash(); }

        bool operator==(const Node& node) const { return state == node.state; }

        std::string toString() const {
            std::ostringstream stream;
            stream << "s: " << state << " g: " << g << " h: " << h << " f: " << f() << " a: " << action << " p: ";
            if (parent == nullptr) {
                stream << "None";
            } else {
                stream << parent->state;
            }
            stream << (open ? " Open" : " Not Open");
            return stream.str();
        }

        /** Index used by the priority queue */
        mutable unsigned int index;
        /** Parent node */
        Node* parent;
        /** Internal state */
        const State state;
        /** Action that led to the current node from the parent node */
        Action action;
        /** Cost from the root node */
        Cost g;
        /** Heuristic cost of the node */
        Cost h;
        /** True if the node is in the open list */
        bool open;
        /** Last iteration when the node was updated */
        unsigned int iteration;
        /** List of all the predecessors that were discovered in the current exploration phase. */
        std::vector<Edge> predecessors;
        /** The action at the root of the tree where this node originally descended from*/
        Action topLevelAction;
        /** The number of parents we haven't yet explored */
        int unexploredParents = 0;
        /** nodes that have preceeded this one*/
        std::vector<Node*> preceeders;
        /** safe label */
        bool isSafe = false;
    };

    class Edge {
    public:
        Edge(Node* predecessor, Action action, Cost actionCost)
                : predecessor{predecessor}, action{action}, actionCost{actionCost} {}

        Node* predecessor;
        const Action action;
        const Cost actionCost;
    };

    /** sweep back through the parent pointers */
    void sweepBackSafety() {
        for (auto currentSafeNode : safeNodes) {
            if (currentSafeNode != nullptr) {
                Node* newFoundSafeNode = currentSafeNode->parent;
                newFoundSafeNode->isSafe = true;
                while (newFoundSafeNode != nullptr) {
                    if (newFoundSafeNode->parent != nullptr && newFoundSafeNode->parent->parent == nullptr) {
                        safeTopLevelActions.push_back(newFoundSafeNode); // safe TLA
                    } else if (newFoundSafeNode->parent != nullptr) {
                        safeNodes.push_back(newFoundSafeNode); // safe node
                    }
                    newFoundSafeNode = newFoundSafeNode->parent;
                }
            }
        }
    }

    //    void sweepBackSafetyOld() {
    //        for (auto currentSafeNode : safeNodes) {
    //            if (currentSafeNode != nullptr) {
    //                Node* newFoundSafeNode = currentSafeNode->parent;
    //                while (newFoundSafeNode != nullptr) {
    //                    newFoundSafeNode->isSafe = true;
    //                    if (newFoundSafeNode->parent != nullptr) {
    //                        if (newFoundSafeNode->parent->parent == nullptr)
    //                            safeTopLevelActionNodes.push_back(newFoundSafeNode);
    //                    } else {
    //                        safeNodes.push_back(newFoundSafeNode);
    //                    }
    //                    newFoundSafeNode = newFoundSafeNode->parent;
    //                }
    //                for (auto copyIt : currentSafeNode->preceeders) {
    //                    Node* newFoundPredNode = copyIt->parent;
    //                    while (newFoundPredNode != nullptr) {
    //                        if (newFoundPredNode->parent != nullptr) {
    //                            if (newFoundPredNode->parent->parent == nullptr)
    //                                safeTopLevelActionNodes.push_back(newFoundPredNode);
    //                        } else {
    //                            safeNodes.push_back(newFoundPredNode);
    //                        }
    //                        newFoundPredNode->isSafe = true;
    //                        newFoundPredNode = newFoundPredNode->parent;
    //                    }
    //                }
    //            } else {
    //                break;
    //            }
    //        }
    //    }

    void learn(const TerminationChecker& terminationChecker) {
        ++iterationCounter;

        // Reorder the open list based on the heuristic values
        openList.reorder(hComparator);

        while (!terminationChecker.reachedTermination() && openList.isNotEmpty()) {
            auto currentNode = popOpenList();
            Node currentNodeLocal = *currentNode;
            currentNode->iteration = iterationCounter;
            currentNodeLocal.iteration = iterationCounter;

            Cost currentHeuristicValue = currentNode->h;
            Cost localCurrentHeruristicValue = currentNodeLocal.h;
            heuristicValues[currentNodeLocal] = localCurrentHeruristicValue;

            // update heuristic actionDuration of each predecessor
            for (auto predecessor : currentNode->predecessors) {
                Node* predecessorNode = predecessor.predecessor;

                if (predecessorNode->iteration == iterationCounter && !predecessorNode->open) {
                    // This node was already learned and closed in the current iteration
                    continue;
                    // TODO Review this. This could be incorrect if the action costs are not uniform
                }

                if (!predecessorNode->open) {
                    // This node is not open yet, because it was not visited in the current planning iteration

                    predecessorNode->h = currentHeuristicValue + predecessor.actionCost;
                    assert(predecessorNode->iteration == iterationCounter - 1);
                    predecessorNode->iteration = iterationCounter;

                    addToOpenList(*predecessorNode);
                } else if (predecessorNode->h > currentHeuristicValue + predecessor.actionCost) {
                    // This node was visited in this learning phase, but the current path is better then the previous
                    predecessorNode->h = currentHeuristicValue + predecessor.actionCost;
                    openList.update(*predecessorNode);
                }
            }
        }
    }

    Node* explore(const State& startState, TerminationChecker& terminationChecker) {
        ++iterationCounter;
        clearOpenList();
        openList.reorder(fComparator);

        safeNodes.clear();
        safeTopLevelActions.clear();

        Planner::incrementGeneratedNodeCount();
        Node*& startNode = nodes[startState];

        if (startNode == nullptr) {
            startNode = nodePool->construct(Node{nullptr, startState, Action(), 0, domain.heuristic(startState), true});
        } else {
            startNode->g = 0;
            startNode->action = Action();
            startNode->predecessors.clear();
            startNode->parent = nullptr;
        }

        startNode->iteration = iterationCounter;
        addToOpenList(*startNode);

        Node* currentNode = startNode;

        while (!terminationChecker.reachedTermination() && !domain.isGoal(currentNode->state)) {
            //            if (domain.safetyPredicate(currentNode->state)) { // try to find nodes which lead to safety
            //                currentNode = popOpenList();
            //                terminationChecker.notifyExpansion();
            //                expandNode(currentNode);
            //            }
            //
            //            if (currentNode == startNode) { // if we can't just do LSS-LRTA*
            //                while (!terminationChecker.reachedTermination() && !domain.isGoal(currentNode->state)) {
            //                    currentNode = popOpenList();
            //                    terminationChecker.notifyExpansion();
            //                    expandNode(currentNode);
            //                }
            //            }
            Node* const currentNode = popOpenList();

            if (domain.isGoal(currentNode->state)) {
                return currentNode;
            }

            terminationChecker.notifyExpansion();
            expandNode(currentNode, startNode);
        }

        return openList.top();
    }

    void checkSafeNode(Node* candidateNode, Node* startNode) {
        if (domain.safetyPredicate(candidateNode->state) &&
                !(candidateNode->state == startNode->state)) { // if the node is safe
            candidateNode->isSafe = true;
            if ((candidateNode->parent != nullptr && candidateNode->parent->parent == nullptr) ||
                    candidateNode->parent == startNode) {
                safeTopLevelActions.push_back(candidateNode);
            } else {
                safeNodes.push_back(candidateNode); // put it with the other safe nodes
            }
        }
    }

    void expandNode(Node* sourceNode, Node* startNode) {
        Planner::incrementExpandedNodeCount();

        for (auto successor : domain.successors(sourceNode->state)) {
            auto successorState = successor.state;

            Node*& successorNode = nodes[successorState];

            if (successorNode == nullptr) {
                successorNode = createNode(sourceNode, successor);
            }

            // If the node is outdated it should be updated.
            if (successorNode->iteration != iterationCounter) {
                successorNode->iteration = iterationCounter;
                successorNode->predecessors.clear();
                successorNode->g = Domain::COST_MAX;
                successorNode->open = false; // It is not on the open list yet, but it will be
                // parent, action, and actionCost is outdated too, but not relevant.
            }
            checkSafeNode(successorNode, startNode);
            if (successorNode->parent != nullptr) { // as long as the parent isn't null
                if (successorNode->parent->parent == nullptr ||
                        successorNode->parent == startNode) { // if we're marking TLAs
                    successorNode->topLevelAction = successorNode->action; // give them their action
                } else { // otherwise we're marking non TLAs
                    successorNode->topLevelAction =
                            successorNode->parent->topLevelAction; // set the node's TLA to its parent's
                }
            }
            // Add the current state as the predecessor of the child state
            successorNode->predecessors.emplace_back(sourceNode, successor.action, successor.actionCost);

            // Skip if we got back to the parent
            if (sourceNode->parent != nullptr && successorState == sourceNode->parent->state) {
                continue;
            }

            // only generate those state that are not visited yet or whose cost value are lower than this path
            Cost successorGValueFromCurrent{sourceNode->g + successor.actionCost};
            if (successorNode->g > successorGValueFromCurrent) {
                successorNode->g = successorGValueFromCurrent;
                successorNode->parent = sourceNode;
                successorNode->action = successor.action;

                if (!successorNode->open) {
                    addToOpenList(*successorNode);
                } else {
                    openList.update(*successorNode);
                }
            }
        }
    }

    Node* createNode(Node* sourceNode, SuccessorBundle<Domain> successor) {
        Planner::incrementGeneratedNodeCount();
        Node* ret = nodePool->construct(Node{sourceNode,
                successor.state,
                successor.action,
                domain.COST_MAX,
                domain.heuristic(successor.state),
                true});
        ret->preceeders.push_back(sourceNode); // keep track of all nodes that have generated this one
        return ret;
    }

    void clearOpenList() {
        openList.forEach([](Node* node) { node->open = false; });
        openList.clear();
    }

    Node* popOpenList() {
        if (openList.isEmpty()) {
            throw MetronomeException("Open list was empty, goal not reachable.");
        }

        Node* node = openList.pop();
        node->open = false;
        return node;
    }

    void addToOpenList(Node& node) {
        node.open = true;
        openList.push(node);
    }

    std::vector<ActionBundle> extractPath(const Node* targetNode, const Node* sourceNode) {
        if (domain.isGoal(targetNode->state) && targetNode->parent->parent == nullptr) {
            return std::vector<ActionBundle>{ActionBundle{targetNode->action, targetNode->g - targetNode->parent->g}};
        }
        if (targetNode == sourceNode) {
            //            LOG(INFO) << "We didn't move:" << sourceNode->toString() << std::endl;
            return std::vector<ActionBundle>();
        }

        std::vector<ActionBundle> actionBundles;
        auto currentNode = targetNode;

        // commit to one action
        //        LOG(INFO) << currentNode->toString() << std::endl;
        if (currentNode->parent->parent == nullptr) {
            //            LOG(INFO) << "root" << std::endl;
        }
        //        LOG(INFO) << currentNode->topLevelAction << std::endl;
        while (currentNode->parent->parent != nullptr) {
            //         The g difference of the child and the parent gives the action cost from the parent
            //  keep going back until we're one away from the root AKA at a TLA
            //            actionBundles.emplace_back(currentNode->action, currentNode->g - currentNode->parent->g);
            currentNode = currentNode->parent;
        }

        if (safeTopLevelActions[0] == nullptr) {
//            LOG(INFO) << "LSS_LRTA* behavior" << std::endl;
            actionBundles.emplace_back(currentNode->action, currentNode->g - currentNode->parent->g);
        } else {
            // do pick a safe top level action
            const Node* safestTLA = findSafestTLA(currentNode->parent);
            if(safestTLA != nullptr) {
                actionBundles.emplace_back(safestTLA->action, safestTLA->g - safestTLA->parent->g);
            } else {
                actionBundles.emplace_back(currentNode->action, currentNode->g - currentNode->parent->g);
            }
        }
        std::reverse(actionBundles.begin(), actionBundles.end());

        return actionBundles;
    }

    Node* findSafestTLA(Node* currentNode) {
        Node* safestTLA = nullptr;
        long int lowestF = std::numeric_limits<long>::max();
        std::vector<Node*> placeOnToOpen;
        int count = 0;
        while (openList.isNotEmpty()) {
            Node* topOfOpen = openList.pop();
            placeOnToOpen.push_back(topOfOpen);
            if (topOfOpen->f() < lowestF && topOfOpen->isSafe && currentNode == topOfOpen->parent) {
                safestTLA = topOfOpen;
                ++count;
                lowestF = topOfOpen->f();
            }
        }
        std::cout << count << std::endl;
        int index = 0;
        while (index < placeOnToOpen.size()) {
            Node* backOnOpen = placeOnToOpen[index];
            openList.push(*backOnOpen);
            ++index;
        }
        openList.reorder(fComparator);
        return safestTLA;
    }

    static int fComparator(const Node& lhs, const Node& rhs) {
        if (lhs.f() < rhs.f())
            return -1;
        if (lhs.f() > rhs.f())
            return 1;
        if (lhs.g > rhs.g)
            return -1;
        if (lhs.g < rhs.g)
            return 1;
        return 0;
    }

    static int hComparator(const Node& lhs, const Node& rhs) {
        if (lhs.h < rhs.h)
            return -1;
        if (lhs.h > rhs.h)
            return 1;
        return 0;
    }

    std::vector<Node*> safeNodes{Memory::NODE_LIMIT};
    const Domain& domain;
    PriorityQueue<Node> openList{Memory::OPEN_LIST_SIZE, fComparator};
    std::unordered_map<State, Node*, typename metronome::Hasher<State>> nodes{};
    std::vector<Node*> safeTopLevelActions{Memory::NODE_LIMIT};

    std::unique_ptr<StaticVector<Node, Memory::NODE_LIMIT>> nodePool{
            std::make_unique<StaticVector<Node, Memory::NODE_LIMIT>>()};
    unsigned int iterationCounter{0};
    std::unordered_map<Node, int, typename metronome::Hasher<Node>> heuristicValues{};
};
}

#endif // METRONOME_SLSSLRTASTAR_HPP
