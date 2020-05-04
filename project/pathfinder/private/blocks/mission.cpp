#include "mission.hpp"


namespace Pathfinder::Nodes
{
	bool ScriptNode::IsValid() const
	{
		return Script != nullptr;
	}

	bool StaticNode::IsValid() const
	{
		return true;
	}

	auto CastNode(const INode::ptr& inode) -> std::tuple<ScriptNode*, StaticNode*>
	{
		if (inode)
		{
			if (auto ptr = dynamic_cast<ScriptNode*>(&*inode))
			{
				return { ptr, nullptr };
			}
			if (auto ptr = dynamic_cast<StaticNode*>(&*inode))
			{
				return { nullptr, ptr };
			}
		}
		throw std::runtime_error("no conversion from inode avaliable");
	}
}
