![](./resources/official_armmbed_example_badge.png)

# Mbed Pixy2 and Pixy2.1 UART non blocking Library Summary

This library is a [Mbed Library](https://os.mbed.com/) that use UART (@ 230Kbps) to communicate with a [Pixy2 smart camera](https://pixycam.com/) using serial IRQ to obtain non blocking functions.

## Authors

- [haarkon](https://github.com/haarkon)
- Wael Hazami
- Theo Le Paih

# Documentation

Pixy2.h contain a fully written DOXYGEN format documentation.
You may find the whole documentation and a FAQ on [PIXY website](https://pixycam.com/)

The pixy 2 (and 2.1) use a request/response algorithm: User send a request, which is then processed by the camera, who then respond to the user.
This cause a (more or less long) delay between the order and the response. Waiting for the response may take a long time (sometime around 1/60 second).
That's why this library is made to be non blocking.

All methods return immediatly with an error code giving to the user hints about what's camera is doing:
- PIXY2::PIXY2_OK   -> Order has been processed and a response can be read
- PIXY2::PIXY2_BUSY -> Camera is processing your order, no response is available
- All other codes   -> An error has occured, no data will be produced

The library stores received data in a large (256 bytes) home made circular buffer.

User can access to response datas by using 2 possibilities :
- In a single structure response, you may either:
  - read the global variable associated with the requested data
  - use a pointer that will be mapped on the reception buffer
- In a multiple structure (either many types and/or many time) response, you must access each type of variable using 2 global variables:
  - an enumerator that indicate how many time the same structure has been found
  - an array of the structure

You may find an example just below.

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
     PIXY2::T_pixy2ErrorCode rCode;            // return Code
     PIXY2::T_pixy2Version *version = nullptr; // Version for Pixy2 (no allocation needed)
     PIXY2::T_pixy2Bloc *bloc = nullptr;       // Easy to use Color Code Bloc (not mandatory, no allocation needed)
 
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
         rCode = cam.pixy2_getVersion (version); // this will set address of "version" somewhere in the reception buffer
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
                 
                 bloc = &cam.Pixy2_blocks[i]; // For a easy usage - still not mandatory - else use cam->Pixy2_blocks[i].<field> instead of bloc-><field> 
                 
                 printf("bloc %d - sig = %d\n", i+1, bloc->pixSignature); // Display block siagnature
                 printf("\tposition : X = %d, Y = %d\n", bloc->pixX, bloc->pixY); // Display block position
                 printf("\tsize : H = %d, W = %d\n", bloc->pixHeight, bloc->pixWidth); // Display block size
             }
             ThisThread::sleep_for(WAIT_TIME_MS); // Only usefull for a printf debug
         } // Here you may check for erroneous responses
     }
 }
``` 
