#include <iostream>
#include <math.h>

using namespace std;


void khat(int x1, int x2, int y1, int y2, float &a, float &b)
{// Y = aX + b
    a = (y2 - y1) / (x2 - x1);
    b = y1 - a * x1;
}

void taqato(int x1, int y1,int r , float a, float b , float &x21, float &x22)
{
    // X= ((+/-)radical(B^2 - 4AC) -B)/2A

    float A, B, C, delta;

    A = a * a + 1;
    B = 2*((a*b) -(a * y1) - x1 );
    C = ((b * b) + (y1 * y1) - (2 * b * y1) + (x1 * x1)) - (r * r);
    delta = B * B - 4 * A * C;

    x21 = (sqrt(delta) - B) / (2 * A);
    x22 = (-sqrt(delta) - B) / (2 * A);


}



int main()
{
    int x1=300, x2=200, y1=300, y2=400 ,r=50;
    float x3, x4, y3, y4;


    if (x1==x2)
    {
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
    }

    else
    {
        float a, b;
        khat(x1, x2, y1, y2, a,b );

        cout<<a<<"\t"<<b<<endl;

        float x31, x32;
        taqato(x1, y1, r, a, b, x31, x32);

        float x41, x42;
        taqato(x2, y2, r, a, b, x41, x42);


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

    cout <<"(" <<x3 <<"\t"<< y3 << ") , ("<< x4<< "\t"<< y4<<")";


}
