#include <LittlePhysics/CollisionBroadPhase.h>
#include <LittlePhysics/CollisionNarrowPhase.h>
#include <queue>
namespace LP {

    void DbvhTree::TestCollision(Index index)
    {
        if (index == IndexNull) return;
        auto& node = m_Nodes[index];
        TestCollision(node.Child[0]);
        TestCollision(node.Child[1]);
        TestCollision2(node.Child[0], node.Child[1]);
    }

    void DbvhTree::TestCollision2(Index indexA, Index indexB)
    {
        if (indexA == IndexNull || indexB == IndexNull)
            return;
        auto& nodeA = m_Nodes[indexA];
        auto& nodeB = m_Nodes[indexB];
        if (nodeA.AaBb.TestOverlap(nodeB.AaBb))
        {
            if (nodeA.body && nodeB.body)
            {
                m_CollisionPairs.push_back({ nodeA.body, nodeB.body });
            }
            else if (nodeA.body)
            {
                TestCollision2(indexA, nodeB.Child[0]);
                TestCollision2(indexA, nodeB.Child[1]);
            }
            else
            {
                TestCollision2(indexB, nodeA.Child[0]);
                TestCollision2(indexB, nodeA.Child[1]);
            }
        }
    }

    void DbvhTree::TestCollision()
    {
        m_CollisionPairs.clear();
        TestCollision(m_Root);
        for (uint32 i = 0; i < m_NodeCount; i++)
        {
            m_Nodes[i].Updated = false;
        }
        // TODO: Make the m_Nodes more compact
    }
    DbvhTree::Index DbvhTree::Update(Index handle, const AABB& aabb)
    {
        auto& node = m_Nodes[handle];
        Body* body = node.body;
        if (aabb.IsIn(node.AaBb))
            return handle;
        // TODO: remove the node and insert without allocate node
        Remove(handle);
        return Insert(body, aabb);
    }
    
    DbvhTree::Index DbvhTree::Insert(Body* body, const  AABB& aabb)
    {
        Index newNodeIndex;
        auto& newNode = AllocateNode(newNodeIndex);

        newNode.AaBb = aabb;
        Vec2 center = (aabb.Max + aabb.Min) * 0.5f;
        newNode.AaBb.Max = (aabb.Max - center) * m_EnlargeFactor + center;
        newNode.AaBb.Min = (aabb.Min - center) * m_EnlargeFactor + center;
        newNode.body = body;
        newNode.Child[0] = -1;
        newNode.Child[1] = -1;
        newNode.Parent = -1;
        newNode.Updated = true;
        newNode.Area = Area(aabb);
        if (m_Root < 0)
        {
            m_Root = newNodeIndex;
        }
        else
        {
            // Step1. Running a DFS to find best node
            float bestCost;
            Index bestNodeIndex;

            struct DFSNode
            {
                Index Index;
                float Cost;
            };

            std::queue<DFSNode> nodes;
            {
                auto& node = m_Nodes[m_Root];
                float unionCost = Area(aabb, node.AaBb);
                float accumulateCost = 0.0f;
                bestCost = unionCost;
                bestNodeIndex = m_Root;
                accumulateCost = unionCost - node.Area;
                if (node.Child[0] != IndexNull)
                    nodes.push({ node.Child[0], accumulateCost });
                if (node.Child[1] != IndexNull)
                    nodes.push({ node.Child[1], accumulateCost });
            }
            while (!nodes.empty())
            {
                DFSNode dfsNode = nodes.front();
                nodes.pop();
                auto& node = m_Nodes[dfsNode.Index];
                float unionCost = Area(aabb, node.AaBb);


                float accumulateCost = dfsNode.Cost + unionCost - node.Area;

                if (unionCost + dfsNode.Cost < bestCost)
                {
                    bestNodeIndex = dfsNode.Index;
                    bestCost = unionCost + dfsNode.Cost;
                }

                float childMinCost = newNode.Area + accumulateCost;
                if (childMinCost < bestCost)
                {
                    if (node.Child[0] != IndexNull)
                        nodes.push({ node.Child[0], accumulateCost });
                    if (node.Child[1] != IndexNull)
                        nodes.push({ node.Child[1], accumulateCost });
                }
            }
            auto& bestNode = m_Nodes[bestNodeIndex];

            // Step2. Create new Node
            Index unionNodeIndex;
            auto& unionNode = AllocateNode(unionNodeIndex);

            unionNode.Parent = bestNode.Parent;
            unionNode.Child[0] = bestNodeIndex;
            unionNode.Child[1] = newNodeIndex;
            //unionNode.AaBb = Union(bestNode.AaBb, newNode.AaBb);
            //unionNode.Area = Area(unionNode.AaBb);
            unionNode.body = nullptr;
            unionNode.ChildIndex = bestNode.ChildIndex;
            unionNode.Updated = true;

            bestNode.Parent = unionNodeIndex;
            bestNode.ChildIndex = 0;

            newNode.Parent = unionNodeIndex;
            newNode.ChildIndex = 1;

            if (unionNode.Parent != IndexNull)
            {
                auto& parentNode = m_Nodes[unionNode.Parent];
                parentNode.Child[unionNode.ChildIndex] = unionNodeIndex;
            }
            else
            {
                m_Root = unionNodeIndex;
            }

            // Step3. Walk back to refit the ancestors
            // TODO: Rotate
            Index refitNodeIndex = unionNodeIndex;
            RefitFrom(refitNodeIndex);
        }


        return newNodeIndex;
    }

    void DbvhTree::Remove(Index handle)
    {
        if (handle == IndexNull) return;
        if (handle >= m_NodeCount) return;
        auto& node = m_Nodes[handle];
        if (node.Parent == IndexNull)
        {
            FreeNode(handle);
            m_Root = IndexNull;
            return;
        }
        // ReConnect
        auto& parentNode = m_Nodes[node.Parent];
        Index recycleIndex1 = handle;
        Index recycleIndex2 = node.Parent;
        if (parentNode.Parent == IndexNull)
        {
            m_Root = parentNode.Child[(node.ChildIndex + 1) % 2];
            m_Nodes[m_Root].Parent = IndexNull;
        }
        else
        {
            auto& parentNode2 = m_Nodes[parentNode.Parent];
            Index neighborIndex = (node.ChildIndex + 1) % 2;
            parentNode2.Child[parentNode.ChildIndex] = parentNode.Child[neighborIndex];
            auto& neighberNode = m_Nodes[parentNode.Child[neighborIndex]];
            neighberNode.Parent = parentNode.Parent;
            neighberNode.ChildIndex = parentNode.ChildIndex;
            

            RefitFrom(parentNode.Parent);

        }
        // Replace node memory
        FreeNode(recycleIndex1);
        FreeNode(recycleIndex2);
    }

    void DbvhTree::RefitFrom(Index index)
    {
        Index refitNodeIndex = index;
        while (refitNodeIndex != IndexNull)
        {
            auto& refitNode = m_Nodes[refitNodeIndex];
            auto& childNode1 = m_Nodes[refitNode.Child[0]];
            auto& childNode2 = m_Nodes[refitNode.Child[1]];
            refitNode.Area = Area(childNode1.AaBb, childNode2.AaBb);
            refitNode.Updated = true;

            // Rotate Procedure
            if (refitNode.Parent != IndexNull)
            {
                auto& refitParent = m_Nodes[refitNode.Parent];
                Index targetNodeChildIndex = (refitNode.ChildIndex + 1) % 2;
                auto& targetNode = m_Nodes[refitParent.Child[targetNodeChildIndex]];
                uint32 childIndex = 0;
                float newArea[2];
                newArea[0] = Area(childNode1.AaBb, targetNode.AaBb);
                newArea[1] = Area(childNode2.AaBb, targetNode.AaBb);
                if (newArea[1] < newArea[0]) childIndex++;
                // Check whether rotate is worth it
                if (newArea[childIndex] < refitNode.Area)
                {
                    childIndex = (childIndex + 1) % 2;
                    // Rotate childNode and targetNode
                    auto index = refitNode.Child[childIndex];
                    refitNode.Child[childIndex] = refitParent.Child[targetNodeChildIndex];
                    m_Nodes[refitNode.Child[childIndex]].Parent = refitNodeIndex;
                    m_Nodes[refitNode.Child[childIndex]].ChildIndex = childIndex;
                    refitParent.Child[targetNodeChildIndex] = index;
                    m_Nodes[refitParent.Child[targetNodeChildIndex]].Parent = refitNode.Parent;
                    m_Nodes[refitParent.Child[targetNodeChildIndex]].ChildIndex = targetNodeChildIndex;
                }
            }

            refitNode.AaBb = Union(m_Nodes[refitNode.Child[0]].AaBb, m_Nodes[refitNode.Child[1]].AaBb);
            refitNode.Area = Area(refitNode.AaBb);
            refitNodeIndex = refitNode.Parent;
        }
    }

    DbvhNode& DbvhTree::AllocateNode(Index& index)
    {
        if (m_FreeNodes.empty())
        {
            index = m_NodeCount;
            m_NodeCount++;
        }
        else
        {
            index = m_FreeNodes.front();
            m_FreeNodes.pop_front();
        }
        return m_Nodes[index];
    }

    DbvhTree::Index DbvhTree::AllocateNode(DbvhNode* node)
    {
        Index index;
        if (m_FreeNodes.empty())
        {
            index = m_NodeCount;
            m_NodeCount++;
        }
        else
        {
            index = m_FreeNodes.front();
            m_FreeNodes.pop_front();
        }
        node = &m_Nodes[index];
        return index;
    }

    void DbvhTree::FreeNode(Index index)
    {
        m_FreeNodes.push_back(index);
    }



}