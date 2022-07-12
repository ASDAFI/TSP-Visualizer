//
// Created by amir.
//

#include "iostream"
#include "vector"
using namespace std;

//////////////// Graph //////////////

Graph :: Graph()
{
    edgesCount = edges.size();
    nodesCount = nodes.size();
}

//////////////// addEdge ////////////

void Graph :: addEdge(Edge* inputEdge)
{
    edges.push_back(inputEdge);
}

//////////////// addNode ////////////

void Graph :: addNode(Node* inputNode)
{
    nodes.push_back(inputNode);
}

int main() {
    cout << "Hello World" << endl ;
}
