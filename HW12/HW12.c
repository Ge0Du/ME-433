#include <stdio.h>
#include "pico/stdlib.h"
#include "cam.h"
void findCOM2D(int *outX, int *outY);

int main()
{
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    //printf("Hello, camera!\n");

    init_camera_pins();
 
    while (true) {
        // uncomment these and printImage() when testing with python 
        char m[10];
        scanf("%s",m);

        setSaveImage(1);
        while(getSaveImage()==1){ }
        convertImage();
        // ─────  NEW: compute full 2D COM ─────
        int comX = 0, comY = 0;
        int comX1 = 0, comY1 = 0;
        
        
        findCOM2D(&comX, &comY);
        findCOMWhite(&comX1, &comY1);
        // note: findCOM2D writes the pixel coordinates into (comX,comY)

        // draw a green dot at (row=comY, col=comX)
        setPixel(comY, comX, 255, 0, 0);
        setPixel(comY1, comX1, 0, 255, 0);
        printImage();
        //printf("%d\r\n",com); // comment this when testing with python
    }
}

