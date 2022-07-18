
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


Position* getCoordinates(int x1, int y1, int x2, int y2, int r)
{
    // This function find coordinates to draw better nodes.
    int x3, y3, x4, y4;

    if(x1 == x2){
        x3=x1;
        x4=x1;

        if (y1>y2)
        {
            y3=y1-r;
            y4=y2+r;

        }
        if (y2>y1)
        {
            y3=y1+r;
            y4=y2-r;
        }
        Position* coordinates = new Position[2];
        coordinates[0] = Position{x3, y3};
        coordinates[1] = Position{x4, y4};

        return coordinates;
    }

    float A, B, C, delta, a, b;

    a = (y2 - y1) / (x2 - x1);
    b = y1 - a * x1;




    A = a * a + 1;

    B = 2*((a*b) -(a * y1) - x1 );
    C = ((b * b) + (y1 * y1) - (2 * b * y1) + (x1 * x1)) - (r * r);
    delta = B * B - 4 * A * C;

    int x31, x32;
    x31 = (sqrt(delta) - B) / (2 * A);
    x32 = (-sqrt(delta) - B) / (2 * A);


    B = 2*((a*b) -(a * y2) - x2 );
    C = ((b * b) + (y2 * y2) - (2 * b * y2) + (x2 * x2)) - (r * r);
    delta = B * B - 4 * A * C;

    int x41, x42;
    x41 = (sqrt(delta) - B) / (2 * A);
    x42 = (-sqrt(delta) - B) / (2 * A);



    if (x1 > x2)
    {
        x3 = x32;
        y3 = a * x32 + b;

        x4 = x41;
        y4 = a * x41 + b;

    } else {
        x3 = x31;
        y3= a * x31 + b;

        x4 = x42;
        y4 = a * x42 + b;
    }

    Position* coordinates = new Position[2];
    coordinates[0] = Position{x3, y3};
    coordinates[1] = Position{x4, y4};

    return coordinates;


}

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
    int x1 = firstNode->getPosition().x, y1 = firstNode->getPosition().y;
    int x2 = secondNode->getPosition().x, y2 = secondNode->getPosition().y;
    float r = Node::radius;

    Position* coordinates =  getCoordinates(x1, y1, x2, y2, r);
    int x3 = coordinates[0].x, y3 = coordinates[0].y;
    int x4 = coordinates[1].x, y4 = coordinates[1].y;

    setlinestyle(SOLID_LINE, 0, Edge::thickness);
    //line(alpha1, beta1, alpha2, beta2);
    line(x3, y3, x4, y4);
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

    Edge edge1(1, &node1, &node2, 1);
    Edge edge2(2, &node2, &node3, 1);
    Edge edge3(3, &node3, &node1, 1);

    g.addEdge(&edge1);
    g.addEdge(&edge2);
    g.addEdge(&edge3);





    g.visualize();


    delay(10000);

    return 0;
}
