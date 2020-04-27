#ifndef PATHFINDER__PATHTREE_HPP
#define PATHFINDER__PATHTREE_HPP

#include <boost/noncopyable.hpp>
#include "common.hpp"


namespace Pathfinder
{
	template<typename T>
	class PathTree final : boost::noncopyable
	{
		struct Node : boost::noncopyable
		{
			using ptr = std::unique_ptr<Node>;
			
			int lvl = 0;
			Node* parent = nullptr;
			std::vector<ptr> children;
			T payload;

		public:
			static ptr New()
			{
				return std::make_unique<Node>();
			}

			Node() = default;

			Node(const T& payload)
				: payload(payload)
			{}

			Node* NewChild(const T& childData)
			{
				auto child   = New();
				auto pointer = &*child;

				child->lvl     = lvl + 1;
				child->parent  = this;
				child->payload = childData;
				
				children.emplace_back(std::move(child));
				return pointer;
			}
		};

	public:

		enum EFillInFlag
		{
			  eFirstIteration = 1 << 0
			, eLastIteration  = 1 << 1
		};

		using pathID   = uint64_t;
		using FOnAdded = std::function<void(T& parent, T& child)>;

	public:

		static constexpr pathID rootID = 1;


		PathTree() : root(Node::New())
		{
			ID2Node[rootID] = &*root;
		}

		void RegisterOnAdded(FOnAdded callback)
		{
			onAdded = callback;
		}

		pathID AppendPath(const T& nextLink, pathID path = rootID)
		{
			auto parent  = GetNodeChecked(path);
			auto child   = parent->NewChild(nextLink);
			auto childID = NextID();
			ID2Node[childID] = child;
			if (onAdded && path != rootID)
			{
				onAdded(parent->payload, child->payload);
			}
			return childID;
		}

		auto GetFullPathByID(pathID path)->std::vector<T>
		{
			auto leaf = GetNodeChecked(path);
			auto list = std::vector<T>(leaf->lvl);
			for (auto i = leaf->lvl - 1; i >= 0; --i)
			{
				list[i] = leaf->payload;
				leaf    = leaf->parent;
			}
			return list;
		}

		template<typename C>
		auto GetFullPathByID(const C& paths)->std::vector<std::vector<T>>
		{
			auto lists = std::vector<std::vector<T>>();
			lists.reserve(paths.size());
			for (auto path : paths)
			{
				lists.emplace_back(GetFullPathByID(path));
			}
			return lists;
		}

		T& GetPathByIF(pathID path)
		{
			return GetNodeChecked(path)->payload;
		}

		const T& GetPathByIF(pathID path) const
		{ CONST_FUNCTION_ENTERY(GetPathByIF(path)); }

		template<typename It, typename Fn>
		void FiilInByLayers(It bgn, It end, Fn clb)
		{
			It iA = bgn;
			It iB = bgn + 1;
			for (; iB != end; ++iA, ++iB)
			{
				auto flag = EFillInFlag(
					  (EFillInFlag::eFirstIteration && iA   == bgn)
					| (EFillInFlag::eLastIteration  && iB+1 == end)
				);
				clb(*iA, *iB, flag);
			}
		}

		template<typename C, typename Fn>
		void FiilInByLayers(C& container, Fn clb)
		{
			FiilInByLayers(container.begin(), container.end(), clb);
		}

	private:
		
		Node* GetNodeChecked(pathID id)
		{
			__func__;
			auto node = GetNode(id);
			CheckNodeFromID(node, id);
			return node;
		}

		const Node* GetNodeChecked(pathID id) const
		{ CONST_FUNCTION_ENTERY(GetNodeChecked(id)); }

		Node* GetNode(pathID id)
		{
			auto itr = ID2Node.find(id);
			auto end = ID2Node.end();
			return itr != end ? itr->second : nullptr;
		}

		const Node* GetNode(pathID id) const
		{ CONST_FUNCTION_ENTERY(GetNode(id)); }

		void CheckNodeFromID(const Node* node, pathID id) const
		{
			if (!node)
			{
				throw std::runtime_error("the path doesn't exists in the tree: " + std::to_string(id));
			}
		}

		pathID NextID()
		{
			return ++IDcounter;
		}

	private:

		typename Node::ptr root;

		typename FOnAdded onAdded;
		
		typename pathID IDcounter = rootID;

		std::map<pathID, Node*> ID2Node;
	};
}


#endif //!PATHFINDER__PATHTREE_HPP
