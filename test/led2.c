#include <stdio.h>
#include <stdint.h>

#define RASPI_FANSHIM_PIN_SPI_SCLK  14
#define RASPI_FANSHIM_PIN_SPI_MOSI  15
#define RASPI_APA102_CLCK_STRETCH 5
int
main()
{
   uint8_t buf[] = {0, 0, 0, 0,           /* SOF */
                   0b11100000 | (1 * 31), /* brightness */
                   0,
                   255,
                   0,
                   ~0, ~0, ~0, ~0};       /* EOF */
   int bufLen = sizeof(buf);
   int i, j;

   /* Set LED color */
   for (i = 0; i < bufLen; ++i) {
      uint8_t byte = buf[i];

      for (j = 0; j < 8; ++j) {
         /* Slice off left-most bit */
         uint8_t bit = (byte & 0x80) > 0;

         if (bit) {
            printf("1");
         }
         else {
            printf("0");
         }

         /* Shift left one bit so we can slice it like a salami */
         byte <<= 1; 
      }
      printf("\n");
   }

   return 0;
}