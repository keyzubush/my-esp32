from PIL import Image
import numpy as np

import sys

#


with open(sys.argv[1], mode='rb') as file: # b is important -> binary
    ba = file.read()

if len(sys.argv) > 2:
    rgb = np.frombuffer(ba, np.uint8).reshape(-1, 96, 3)
    image = Image.fromarray(rgb, mode = "RGB")
else:
    gr = np.frombuffer(ba, np.uint8).reshape(-1, 96)
    image = Image.fromarray(gr, mode = "L")
image.show()

print(image.getpixel((0,0)))
