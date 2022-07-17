
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

void Edge::visualize() {



    int alpha1 = firstNode->getPosition().x;
    int beta1 = firstNode->getPosition().y;
    int alpha2 = secondNode->getPosition().x;
    int beta2 = secondNode->getPosition().y;
    int r = Node::radius;

    int x1, y1, x2, y2;

    if(alpha1 == alpha2){
        if(beta1 > beta2){
            swap(beta1, beta2);
        }
        x1 = alpha1;
        x2 = alpha2;

        y1 = beta1 + r;
        y2 = beta2 - r;
    }
    else {
        float m = (beta2 - beta1) / (alpha2 - alpha1);
        float b = beta1 - m * alpha1;

        if (alpha2 < alpha1) {
            swap(alpha1, alpha2);
            swap(beta1, beta2);
        }
        float delta1 = pow(-2 * alpha1 + 2 * b * m - 2 * beta1 * m, 2) -
                       4 * (m + 1) * (pow(alpha1, 2) + pow(b, 2) + pow(beta1, 2) - pow(r, 2) - 2 * beta1 * b);
        x1 = (-(-2 * alpha1 + 2 * b * m - 2 * beta1 * m) + sqrt(delta1)) / (2 * (pow(m, 2) + 1));
        y1 = m * x1 + b;

        float delta2 = pow(-2 * alpha2 + 2 * b * m - 2 * beta2 * m, 2) -
                       4 * (m + 1) * (pow(alpha2, 2) + pow(b, 2) + pow(beta2, 2) - pow(r, 2) - 2 * beta2 * b);
        x2 = (-(-2 * alpha2 + 2 * b * m - 2 * beta2 * m) - sqrt(delta2)) / (2 * (pow(m, 2) + 1));
        y2 = m * x2 + b;
    }
    setlinestyle(SOLID_LINE, 0, Edge::thickness);
    //line(alpha1, beta1, alpha2, beta2);
    line(x1, y1, x2, y2);
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
    void visualizeNodes();
    void visualizeEdges();
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


void Graph::visualizeNodes() {
    for(int i = 0; i < nodesCount; i++) {
        nodes[i]->visualize();
    }
}

void Graph::visualizeEdges() {
    for(int i = 0; i < edgesCount; i++) {
        edges[i]->visualize();
    }
}

void Graph::visualize() {
    visualizeNodes();
    visualizeEdges();
}

////////////////////////////////////////////////// Class Graph - End

///////////////////////Rescaling///////////////////
#include "math.h"

int X = 600, Y = 600;

void Rescaling(double Nodesposition[nodesCount][2])
{
    double minX = Nodesposition[0][0], minY = Nodesposition[0][1];

    for (int i = 1; i < nodescount; i++)
    {
        minX = Nodesposition[i][0] < minX ? Nodesposition[i][0] : minX;
        minY = Nodesposition[i][1] < minY ? Nodesposition[i][1] : minY;
    }

    if (minX < 0)
    {
        for (int i = 0; i < nodescount; i++)
        {
            nodesposition[i][0] += fabs(minX);
        }
    }

    if (minY < 0)
    {
        for (int i = 0; i < nodescount; i++)
        {
            nodesposition[i][1] += fabs(minY);
        }
    }
    
    double maxX = Nodesposition[0][0], maxY = Nodesposition[0][1];
    
    for (int i = 1; i < nodescount; i++)
    {
        maxX = Nodesposition[i][0] > maxX ? Nodesposition[i][0] : maxX;
        maxY = Nodesposition[i][1] > maxY ? Nodesposition[i][1] : maxY;
    }

    double tx, ty;

    tx = X/(maxX - minX);
    ty = Y/(maxY - minY);

    for (int i = 0; i < nodescount; i++)
    {
        Nodesposition[i][0] *= tx;
        Nodesposition[i][1] *= ty;
    }
}
//////////////////////////end/////////////////////////////


int main() {
    cout << "Hello world!" << endl;
    initGraph();

    Node node1(1);
    Node node2(2);
    Node node3(3);

    node1.setPosition(Position{300, 300});
    node2.setPosition(Position{200, 400});
    node3.setPosition(Position{400, 400});


    Graph g;
    g.addNode(&node1);
    g.addNode(&node2);
    g.addNode(&node3);

    //Edge edge1(1, &node1, &node2, 1);
    Edge edge2(2, &node2, &node3, 1);
    Edge edge3(3, &node3, &node1, 1);

    //g.addEdge(&edge1);
    g.addEdge(&edge2);
    g.addEdge(&edge3);





    g.visualize();


    delay(10000);

    return 0;
}
