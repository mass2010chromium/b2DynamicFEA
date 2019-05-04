#include "customDFS.h"

DFSTreeNode* DFS(GraphNode* g) {
	if (g == nullptr) return nullptr;
	
	DFSTreeNode* root = new DFSTreeNode();
	root->nodeValue = g;
	g->_visited = true;
	
	list<DFSTreeNode*> frontier;
	frontier.push_back(root);
	
	while (!frontier.empty()) {
		DFSTreeNode* start = frontier.front();
		frontier.pop_front();
		for (GraphNode* childNode : start->nodeValue->getNeighbors()) {
			if (childNode == nullptr || childNode->_visited) continue;
			childNode->_visited = true;
			DFSTreeNode* newNode = new DFSTreeNode();
			newNode->nodeValue = childNode;
			start->children.push_back(newNode);
			frontier.push_back(newNode);
		}
	}
	
	resetGraphNodes(root);
	return root;
}

void resetGraphNodes(DFSTreeNode* treeNode) {
	treeNode->nodeValue->_visited = false;
	for (DFSTreeNode* t : treeNode->children) {
		resetGraphNodes(t);
	}
}