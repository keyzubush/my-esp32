from PIL import Image
import numpy as np


with open("img.94", mode='rb') as file: # b is important -> binary
    ba = file.read()

rgb565 = np.frombuffer(ba, np.uint16).byteswap().reshape(-1, 96)

red   = ((rgb565 & 0b1111100000000000) >> 8).astype(np.uint8)
green = ((rgb565 & 0b0000011111100000) >> 3).astype(np.uint8)
blue  = ((rgb565 & 0b0000000000011111) << 3).astype(np.uint8)

o = np.dstack([red, green, blue])

image = Image.fromarray(o, mode = "RGB")
image.show()
image = Image.fromarray(np.clip((red.astype(np.int16) * 2 - green - blue), 0, None).astype(np.uint8), mode = "L")
image.show()
image = Image.fromarray((np.sum(o, axis = 2) // 3).astype(np.uint8), mode = "L")
image.show()

print(image.getpixel((0,0)))
