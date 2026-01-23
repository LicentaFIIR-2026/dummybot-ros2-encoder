#!/usr/bin/env python3

from PIL import Image
import os

# Dimensiuni hartă
width, height = 80, 80

# Creează o imagine albă (255 = spațiu liber)
image = Image.new('L', (width, height), 255)  # 'L' = grayscale

# Salvează ca .pgm
output_dir = os.path.expanduser('~/amr2AX_ws/src/amr2ax_nav2/maps')
os.makedirs(output_dir, exist_ok=True)  # Creează directorul dacă nu există
output_path = os.path.join(output_dir, 'demo_map_4x4.pgm')
image.save(output_path)  # Extensia .pgm specifică formatul implicit

print(f'Harta salvată la: {output_path}')