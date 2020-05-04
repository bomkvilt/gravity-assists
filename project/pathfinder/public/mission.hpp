#ifndef PATHFINDER__MISSION_HPP
#define PATHFINDER__MISSION_HPP

#include "interfaces/ephemerides.hpp"


namespace Pathfinder::Nodes
{
	struct INode
	{
		struct InParams
		{
		public: // body relative velocities in solar system frame
			const FVector& W0; // [m/s] - departure
			const FVector& W1; // [m/s] - arrival
		};

		struct OutParams
		{
		public:
			float Impulse    = 0; // [m/s] - impulse of speed the node inside
			float Mismatch   = 0; // [m/s] - mismatch of in/out velocity
			float Correction = 0; // [-] - 
		};

		using ptr = std::shared_ptr<INode>;

	public:
		
		virtual bool IsValid() const = 0;
		virtual auto Check(const InParams& in, bool bGenCorrection = false) const->std::tuple<OutParams, bool> = 0;
	};

	// Node with a scripted non-static object
	// \note:	To find a flight to the object Pathfinder's algorithm scans 
	//			presented time rage [t0, t1] with specified time step. So, if 
	//			the object is static, it's more effective to use a StaticNode 
	//			type instead.
	struct ScriptNode : public INode
	{
		Ephemerides::IEphemerides::ptr Script;

		bool IsValid() const override;


		using ptr = std::shared_ptr<ScriptNode>;
	};

	// Node represents a static point in 3D space
	struct StaticNode : public INode
	{
		FVector R;

		bool IsValid() const override;


		using ptr = std::shared_ptr<StaticNode>;
	};

	// Casts node to one of two sutable derived class
	// \note:	one of the pointer will allways be non-nulled
	// \note:	if both of conversions failed - throw std::runtime_error()
	auto CastNode(const INode::ptr& inode)->std::tuple<ScriptNode*, StaticNode*>;
}


namespace Pathfinder
{
	struct Mission
	{
		using BurnNodeFactory = std::function<Nodes::StaticNode::ptr()>;

	public:
		std::vector<Nodes::INode::ptr> nodes;
		FReal normalFlyPeriodFactor = 0;
		FReal points_f0 = 0;
		FReal timeStep = 0;
		FReal timeTol = 0;
		FReal GM = 0;
		FReal t0 = 0;

		FReal burnArcFraction = 0.5;
		FReal minMinimisationDelta = 0.1;
		Int32 maxMinimisationIters = 100;
		BurnNodeFactory burnNodeFactory;
	};
}


#endif //!PATHFINDER__MISSION_HPP
