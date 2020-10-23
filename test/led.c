#include <stdio.h>
#include <stdint.h>

int
main()
{ 
   uint8_t color[] = {0b11100000 | ((8) & 0b00011111) /* brightness */,
                    0 /* red */,
                    255 /* green */,
                    0 /* blue */};
   int i, j;

   /* Set LED color */
   for (i = 0; i < 4; ++i) {
      uint8_t byte = color[i];

      for (j = 0; j < 8; ++j) {
         if ((byte & (1 << (7 - j))) > 0) {
            printf("1 ");
         }
         else {
            printf("0 ");
         }
      }
      printf("\n");
   }
}
