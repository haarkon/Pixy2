![](./resources/official_armmbed_example_badge.png)

# Pixy2 Library Summary

This library is a Mbed Library (https://os.mbed.com/) that use UART (@ 230Kbps) to communicate with a Pixy2 smart camera (https://pixycam.com/) using serial IRQ to avoid blocking functions.

## Authors

- [haarkon](https://github.com/haarkon)
- Wael Hazami
- Theo Le Paih

# Documentation

Pixy2.h contain a fully written DOXYGEN format documentation.

# Pixy2 usage example

The example shows you how to use the Pixy2 Library.

 ```c++
 #include "mbed.h"
 #include "pixy2.h"
 
 #define WAIT_TIME_MS 100ms 
 DigitalOut led1(LED1);
 PIXY2 cam (PD_5, PD_6, 230000);
 
 int main()
 {
     PIXY2::T_pixy2ErrorCode rCode;  // return Code
     PIXY2::T_pixy2Version *version; // Version for Pixy2 (no allocation needed)
     PIXY2::T_pixy2Bloc *bloc;       // Easy to use Color Code Bloc (not mandatory, no allocation needed)
 
     printf("\nPixy running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
 
     printf("Playing with RGB LED : ");
 
     do {
         // switching on red led
         rCode = cam.pixy2_setLED (0xFF,0,0);
     } while (rCode == PIXY2_BUSY); // waiting for PIXY2 acknowledge (ie : order processed)
 
     // here you may check if return code indicate an erroneous response
 
     ThisThread::sleep_for(500ms);  // to allow visual detection 
 
     do {
         // Switching off red led
         rCode = cam.pixy2_setLED (0,0,0);
     } while (rCode == PIXY2_BUSY); // waiting for acknowledge (ie : order processed)
 
     // here you may check if return code indicate an erroneous response
 
     printf("done\n");
 
     printf("Reading Version : ");
     do {
         // Asking for Pixy2 Version
         rCode = cam.pixy2_getVersion (&version); //address of "version" will be somewhere in the reception buffer
     } while (rCode == PIXY2_BUSY); // waiting for acknowledge (ie : order processed)
 
     // here you may check if return code indicate an erroneous response
 
     printf("done\n");
 
     printf("Pixy : %s (HW : %d) - FW : %d.%d.%d\n", version->pixHFString, version->pixHWVersion, version->pixFWVersionMaj, version->pixFWVersionMin, version->pixFWBuild);
     
     // one must have already set signature with pixyMon2 to track colors
     printf("\nNow Tracking Colors\n");
 
     while (true)
     {
         // Ordering to track all color signature
 
         // As tracking may take some time we use the main loop with a non blocking function to allow other task to be performed while camera is processing the image
         // Order is sent once, then function will return PIXY2_BUSY until color tracking result are made available
         if (cam.pixy2_getBlocks(255, 10) == PIXY2_OK) {
 
             led1 = !led1; //visual debug (not mandatory)
 
             // Displaying number of detected signature blocks
             printf("\nfound : %d blocs\n",cam.Pixy2_numBlocks);
 
             // Parsing thru signature blocs
             for (int i=0; i < cam.Pixy2_numBlocks; i++) {
                 
                 bloc = &cam.Pixy2_blocks[i]; // For a ease usage - still not mandatory - else use cam->Pixy2_blocks[i].<field> to access data 
                 
                 printf("bloc %d - sig = %d\n", i+1, bloc->pixSignature); // Display block siagnature
                 printf("\tposition : X = %d, Y = %d\n", bloc->pixX, bloc->pixY); // Display block position
                 printf("\tsize : H = %d, W = %d\n", bloc->pixHeight, bloc->pixWidth); // Display block size
             }
             ThisThread::sleep_for(WAIT_TIME_MS); // Only usefull for a debug
         } // Here you may check for erroneous responses
     }
 }
``` 
