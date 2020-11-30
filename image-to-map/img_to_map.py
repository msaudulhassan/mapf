from PIL import Image
import numpy as np
from pathlib import Path

image_filename = 'image.bmp'
output_filename = str(Path(image_filename).stem) + ".txt"

with Image.open(image_filename) as im:
	px = im.load()

width, height = im.size
binary_image = [['@']*width for i in range(height)]

for x in range(width):
	for y in range(height):
		r, g, b = px[x,y]
		if r > 128 and g > 128 and b > 128:
			binary_image[y][x] = '.'

binary_image_array = np.array(binary_image)

file = open(output_filename, 'w')

file.write(f"{height} {width}\n")

for row in binary_image:
	row_string = ''
	for col in row:
		row_string += str(col)
	file.write(f"{row_string}\n")

file.close()
