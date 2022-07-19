# include "iostream"
# include "graphics.h"


using namespace std;


struct Point{
    int x;
    int y;
};

class Screen {
    private:
        int width, height;
    public:
        Screen(int inputWidth, int inputHeight) {    
            width = inputWidth;
            height = inputHeight;
        }
        int getWidth() {
            return width;
        }
        int getHeight() {
            return height;
        }    

};

