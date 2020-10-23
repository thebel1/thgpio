#include <stdio.h>
#include <stdint.h>

int main()
{
   uint8_t buf[] = {0, 0, 0, 0,               /* SOF */
                      255,                    /* brightness */
                      0,
                      255,
                      0,
                      1, 1, 1, 1};              /* EOF */

   size_t number_of_bytes = sizeof(buf);
   size_t remaining_bits = (number_of_bytes * 8) % 8;
   if (remaining_bits > 0)
   {
      ++number_of_bytes;
   }

   for (int i = 0; i < number_of_bytes; ++i)
   {
      const uint8_t byte = buf[i];
      uint8_t bits = 8;
      if (i == number_of_bytes - 1)
      {
         bits = remaining_bits;
      }

      for (int j = 0; j < 8; ++j)
      {
         printf("%d", (byte & (1 << (7 - j))) > 0);
      }
      printf("\n");
   }

   return 0;
}