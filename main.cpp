# include "iostream"
# include "vector"
# include "graphics.h"
# include "stdlib.h"
# include "math.h"

using namespace std;

struct Position {
    int x;
    int y;
};

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

};


int Node::radius = 9;
int Node::thickness = Node::radius / 10 + 1;

Node::Node(int inputId)
{
    id = inputId;
}
void Node::setPosition(Position inputPosition) {
    position = inputPosition;
}
int Node::getId() {
    return id;
}
Position Node::getPosition() {
    return position;
}
void Node::visualize() {
    circle(position.x, position.y, Node::radius);

}



class Edge {
    private:
        int id;
        Node* firstNode;
        Node* secondNode;
        double weight;
    public:
        static int thickness;
        Edge(int inputId, Node &inputNode1, Node &inputNode2, double inputWeight);
        void setWeight(double inputWeight);
        double getWeight();
        Node* getFirstNode();
        Node* getSecondNode();
        void visualize();

};


int Edge::thickness = Node::thickness;
Edge::Edge(int inputId, Node &inputNode1, Node &inputNode2) {
    id = inputId;
    firstNode = &inputNode1;
    secondNode = &inputNode2;
    weight = 0;
}
void Edge::setWeight(double inputWeight) {
    weight = inputWeight;
}
double Edge::getWeight() {
    return weight;
}
Node* Edge::getFirstNode() {
    return firstNode;
}
Node* Edge::getSecondNode() {
    return secondNode;
}
void Edge::visualize() {
    float m = (secondNode->getPosition().y - firstNode->getPosition().y) / (secondNode->getPosition().x - firstNode->getPosition().x);
    float b = firstNode->getPosition().y - m * firstNode->getPosition().x;

    int alpha1 = firstNode->getPosition().x;
    int beta1 = firstNode->getPosition().y;
    int alpha2 = secondNode->getPosition().x;
    int beta2 = secondNode->getPosition().y;
    int r = Node::radius;


    if(alpha2 < alpha1) {
        swap(alpha1, alpha2);
        swap(beta1, beta2);
    }
    float delta1 = pow(-2 * alpha1 + 2 * b * m - 2 * beta1 * m, 2) - 4 * (m + 1) * (pow(alpha1, 2) + pow(b, 2)
            + pow(beta1, 2) - pow(r, 2) - 2 * beta1 * b) ;
    float delta2 = pow(-2 * alpha2 + 2 * b * m - 2 * beta2 * m, 2) - 4 * (m + 1) * (pow(alpha2, 2) + pow(b, 2)
            + pow(beta2, 2) - pow(r, 2) - 2 * beta2 * b) ;

    int x1 = (-(-2 * alpha1 + 2 * b * m - 2 * beta1 * m) + sqrt(delta1)) / (2 * (pow(m, 2) + 1));
    int y1 = m*x1 + b;

    int x2 = (-(-2 * alpha2 + 2 * b * m - 2 * beta2 * m) - sqrt(delta2)) / (2 * (pow(m, 2) + 1));
    int y2 = m*x2 + b;

    setlinestyle(SOLID_LINE, 0, Edge::thickness);
    line(x1, y1, x2, y2);
}