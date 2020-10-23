import sys
import math

sys.stdout.write("{")
for x in range(0, 256):
    y = int(math.sin(math.pi * x / 255) * 255)
    sys.stdout.write("%d%s" % (y, ",\t" if x != 255 else ""))
    if (x + 1) % 10 == 0 and x != 255:
        sys.stdout.write("\n")
sys.stdout.write("};\n")