# include "iostream"
# include "graphics.h"
# include "math.h"


using namespace std;


struct Point{
    double x;
    double y;
};


// Screen
class Screen {
    private:
        int width;  // width of screen
        int height; // height of screen
    public:
        Screen(int inputWidth, int inputHeight);
        int getWidth() ;
        int getHeight();
        void initGraph();
        void clear();
        void drawLine(Point point1, Point point2);
        void drawCircle(Point point, int radius);
        void drawLineBetween2Circles(Point center1, Point center2, double radius);

};


Screen::Screen(int inputWidth, int inputHeight) {    
            /* Setup Screen  */
    width = inputWidth;
    height = inputHeight;
    }
int Screen::getWidth() {
    return width;
}
int Screen::getHeight() {
    return height;
}
void Screen::initGraph() {
    int gd = DETECT, gm;
    initgraph(&gd, &gm, NULL);
    // init graph (build GUI)
}
void Screen::clear() {
    // clear screen
    cleardevice();
}
void Screen::drawLine(Point point1, Point point2) {
    // draw line
   line(point1.x, point1.y, point2.x, point2.y);
}
void Screen::drawCircle(Point point, int radius) {
    // draw circle
    circle(point.x, point.y, radius);
}
void Screen::drawLineBetween2Circles(Point center1, Point center2, double radius) {
    // draw line between 2 circles
    
    Point point1, point2;
    // calculation
    if(center1.x  == center2.x){
        point1.x = center1.x ;
        point2.x =center1.x ;

        if (center1.y > center2.y)
        {
            point1.y = center1.y - radius;
            point2.y = center2.y + radius;

        }
        if (center2.y > center1.y)
        {
            point1.y = center1.y + radius;
            point2.y = center2.y - radius;
        }
        

    } else {
        float A, B, C, delta, a, b;
        
        a = (center2.y - center1.y) / (center2.x - center1.x);
        b = center1.y - a * center1.x;
        A = a * a + 1;

        // find probabilies for x point 1
        int x11, x12;
        
        
        B = 2*((a*b) - (a * center1.y) - center1.x );
        C = ((b * b) + (center1.y * center1.y)  - (2 * b * center1.y) + (center1.x * center1.x)) - (radius * radius);
        delta = B * B - 4 * A * C;
        
        x11 = (sqrt(delta) - B) / (2 * A);
        x12 = (-sqrt(delta) - B) / (2 * A);

        // find probabilies for x point 2
        
        int x21, x22;
        
        B = 2*((a*b) - (a * center2.y) - center2.x );
        C = ((b * b) + (center2.y * center2.y)  - (2 * b * center2.y) + (center2.x * center2.x)) - (radius * radius);

        delta = B * B - 4 * A * C;
        x21 = (sqrt(delta) - B) / (2 * A);
        x22 = (-sqrt(delta) - B) / (2 * A);


        /// find points
        if (center1.x > center2.x)

        {
            point1.x = x12;
            point1.y = a * x12 + b;

            point2.x = x21;
            point2.y = a * x21 + b;

        } else {
            point1.x = x11;
            point1.y = a * x11 + b;

            point2.x = x22;
            point2.y = a * x22 + b;
        }


    }
    //

    // draw line
    
    this->drawLine(point1, point2);
}



//


// Node

class Node {

    private:
        int id;
        Point point;
        Point position;

    public:

        static int radius;
        static int thickness;
        Node(int inputId);
        int getId();
        void setPoint(Point inputPoint);
        Point getPoint();
        void setPosition(Point inputPosition);
        Point getPosition();
        void visualize(Screen* screen);

};


int Node::radius = 9;
int Node::thickness = Node::radius / 10 + 1;

Node::Node(int inputId)
{
	id=inputId;
}
int Node::getId()
{
    return id;
}
void Node::setPoint(Point inputPoint)
{
    point = inputPoint;
}
Point Node::getPoint()
{
    return point;
}
void Node::setPosition(Point inputPosition)
{
    position = inputPosition;
}
Point Node::getPosition()
{
    return position;
}
void Node::visualize(Screen* screen)
{
    // draw node
    screen->drawCircle(position, Node::radius);
}

// 

class Edge {
    private:
        int id;
        Node* firstNode;
        Node* secondNode;
        double weight;
        Screen* screen;
    public:
        static int thickness;
        Edge(int inputId, Node* inputNode1, Node* inputNode2, double inputWeight);
        void setWeight(double inputWeight);
        double getWeight();
        Node* getFirstNode();
        Node* getSecondNode();
        void visualize(Screen* screen);

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
void Edge::visualize(Screen* screen){
    // draw edge
    Point point1 = firstNode->getPosition();
    Point point2 = secondNode->getPosition();
    screen->drawLineBetween2Circles(point1, point2, Node::radius);
}



//


int main() {
    Screen screen(800, 600);
    screen.initGraph();
    screen.drawCircle(Point{300, 300}, 10);
    screen.drawCircle(Point{170, 60}, 10);
    screen.drawLineBetween2Circles( Point{300, 300}, Point{170, 60}, 10);
    getch();
}