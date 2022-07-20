# include "iostream"
# include "graphics.h"


using namespace std;


struct Point{
    int x;
    int y;
};


// Screen
class Screen {
    private:
        int width;  // width of screen
        int height; // height of screen
    public:
        Screen(int inputWidth, int inputHeight) {    
            /* Setup Screen  */
            width = inputWidth;
            height = inputHeight;
        }
        int getWidth() {
            return width;
        }
        int getHeight() {
            return height;
        }

        void initGraph() {
            int gd = DETECT, gm;
            initgraph(&gd, &gm, NULL);
            // init graph (build GUI)
        }
        void drawLine(Point point1, Point point2) {
            // draw line
            line(point1.x, point1.y, point2.x, point2.y);
        }

        void drawCircle(Point point, int radius) {
            // draw circle
            circle(point.x, point.y, radius);
        }


};
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