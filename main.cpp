
# include "iostream"
# include "vector"
# include "graphics.h"
# include "stdlib.h"

using namespace std;

struct Position {
    int x;
    int y;
};

////////////////////////////////////////////////// Class Node - Start

class Node {

private:
        int id;
        Position position;
    public:
        static int radius;
        static int thickness;
        Node(int inputId);
        void setPosition(Position inputPosition);
        int getId();
        Position getPosition();
        void visualize();
        Node copy();
};

int Node::radius = 9;
int Node::thickness = Node::radius / 10 + 1;
int Node::(int inputId);
{
	id=inputId;
}
void Node::setPosition(Position inputPosition);
{
	position=inputPosition
}
int Node::getId()
{
	return id;
}
Position Node::getPosition()
{
	return position
}
////////////////////////////////////////////////// Class Node - End


////////////////////////////////////////////////// Class Edge - Start

class Edge {
    private:
        int id;
        Node* firstNode;
        Node* secondNode;
        double weight;
    public:
        static int thickness;
        Edge(int inputId, Node* inputNode1, Node* inputNode2, double inputWeight);
        void setWeight(double inputWeight);
        double getWeight();
        Node* getFirstNode();
        Node* getSecondNode();
        void visualize();

};

//
int Edge::thickness = Node::thickness;

Edge(int inputId, Node* inputNode1, Node* inputNode2, double inputWeight)
{
	id = inputId;
	firstNode = inputNode1;
	secondNode = inputNode2;
	Weight = inputWeight;
}
        
void setWeight(double inputWeight) 
{
    weight = inputWeight;  
}

double getWeight() 
{
	return weight;  
}
Node* getFirstNode(int firstNode)
{
	return firstNode;
}
Node* getSecondNode(int secondNode)
{
	return secondNode;
}
////////////////////////////////////////////////// Class Edge - End


////////////////////////////////////////////////// Class Graph - Start

class Graph {
    vector<Edge*> edges;
    vector<Node*> nodes;
    int edgesCount;
    int nodesCount;
public:
    Graph();
    void addEdge(Edge* inputEdge);
    void addNode(Node* inputNode);
    vector<Edge*> getEdges();
    vector<Node*> getNodes();
    void visualize();

};

Graph::Graph()
{
    edgesCount = 0;
    nodesCount = 0;
}

void Graph::addEdge(Edge* inputEdge)
{
    edges.push_back(inputEdge);
    edgesCount ++;
    
}

void Graph::addNode(Node* inputNode)
{
    nodes.push_back(inputNode);
    nodesCount ++;
 
}



////////////////////////////////////////////////// Class Graph - End

int main() {
    cout << "Hello world!" << endl;
    getch();
    return 0;
}
