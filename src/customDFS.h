#pragma once

#include <vector>
#include <list>

using std::vector;
using std::list;

class DFSTreeNode;

class GraphNode {
	const virtual vector<GraphNode*> getNeighbors() = 0;
	bool _visited = false;

	friend DFSTreeNode* DFS(GraphNode* g);
	friend void resetGraphNodes(DFSTreeNode* treeNode);
};

class DFSTreeNode {
	DFSTreeNode() {
		children = vector<DFSTreeNode*>();
		nodeValue = nullptr;
	}
	
	vector<DFSTreeNode*> children;
	GraphNode* nodeValue;
	
	friend DFSTreeNode* DFS(GraphNode* g);
	friend void resetGraphNodes(DFSTreeNode* treeNode);
};