
# include "iostream"
# include "vector"
# include "graphics.h"
# include "stdlib.h"

using namespace std;

void initGraph(){
    int gd = DETECT, gm;

    initgraph(&gd, &gm, NULL);
}


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

Node::Node(int inputId)
{
	id=inputId;
}
void Node::setPosition(Position inputPosition)
{
	position=inputPosition;
}
int Node::getId()
{
	return id;
}
Position Node::getPosition()
{
	return position;
}

void Node::visualize() {
    circle(position.x, position.y, Node::radius);

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

Edge::Edge(int inputId, Node* inputNode1, Node* inputNode2, double inputWeight)
{
	id = inputId;
	firstNode = inputNode1;
	secondNode = inputNode2;
	weight = inputWeight;
}
        
void Edge::setWeight(double inputWeight)
{
    weight = inputWeight;
}

double Edge::getWeight()
{
	return weight;
}
Node* Edge::getFirstNode()
{
	return firstNode;
}
Node* Edge::getSecondNode()
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
    initGraph();
    Node node1(1);
    node1.setPosition(Position{100, 100});
    node1.visualize();
    getch();
    return 0;
}
