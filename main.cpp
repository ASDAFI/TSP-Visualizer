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


};
//

