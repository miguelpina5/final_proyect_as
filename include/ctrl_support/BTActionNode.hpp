#ifndef BTACTIONNODE_HPP
#define BTACTIONNODE_HPP

#include <behaviortree_cpp_v3/action_node.h>

// Clase base para tus nodos BT
class BTActionNode : public BT::SyncActionNode
{
public:
  BTActionNode(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}
};

#endif  // BTACTIONNODE_HPP