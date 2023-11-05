import matplotlib.pyplot as plt
from PIL import Image

# Load the PGM image using the Pillow library
pgm_image = Image.open('/home/gonk/mren203_ws/src/mren203_package/scripts/contour/mymap.pgm')

# Convert the image to black and white
bw_image = pgm_image.convert('L')

# Create the plot
fig, axs = plt.subplots(1, 2, figsize=(8, 4))
axs[0].imshow(pgm_image)
axs[0].set_title('Original')
axs[1].imshow(bw_image, cmap='gray')
axs[1].set_title('Black and White')

# Display the plot
plt.show()
