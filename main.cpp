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

