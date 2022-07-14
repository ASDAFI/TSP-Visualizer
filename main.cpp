#include <iostream>
#include <math.h>

using namespace std;


double khat(int x1, int x2, int y1, int y2)
{// Y = aX + b
    double a, b;
    a = (y2 - y1) / (x2 - x1);
    b = y1 - a * x1;
    return a, b;
}

double taqato(int x1, int y1,int r , double a, double b)
{
    double x21, x22, y21, y22;
    double A, B, C, delta;
    A = a * a + 1;
    B = -(a * y1 + 2 * x1);
    C = (b * b + y1 * y1 - b * y1 + x1*x1) - (r * r);
    delta = B * B - 4 * A * C;
 
    x21 = (sqrt(delta) - B) / 2 * A;
    x22 = (-sqrt(delta) - B) / 2 * A;
    y21 = a * x21 + b;
    y22 = a * x22 + b;

    return x21, x22;
}



int main()
{
    int x1=100, x2=-100, y1=100, y2=-100 ,r=50;
    double x3, x4, y3, y4;

    double a, b = khat(x1, x2, y1, y2);
    double x31,x32=taqato(x1, y1,r ,a, b);
    double x41,x42=taqato(x2, y2,r ,a, b);

    if (x1 > x2)
    {
        x3 = x32;
        y3 = a * x32 + b;

        x4 = x41;
        y4 = a * x41 + b;

    }
    if (x2 > x1)
    {
        x3 = x31;
        y3= a * x31 + b;

        x4 = x42;
        y4= a * x42 + b;
    }

}