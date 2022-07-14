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
        
void setWeight(double inputWeight) 
{
    Weight = inputWeight;  
}
Edge(int inputId, Node* inputNode1, Node* inputNode2, double inputWeight)
{
	id = inputId;
	firstNode = inputNode1;
	secondNode = inputNode2;
	Weight = inputWeight;
}

double getWeight() 
{
	return Weight;  
}
void getFirstNode(int firstNode)
{
	cout<<firstNode;
}
void getSecondNode(int secondNode)
{
	cout<<secondNode;
}


};

//
int Edge::thickness = Node::thickness;


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


////////////////////////////////////////////////// Class Graph - End

int main() {
	Edge obj;
	cin>>int i;
	obj.setWeight(i);
    cout << "Hello world!" << endl;
    getch();
    return 0;
}