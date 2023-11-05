import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from scipy.interpolate import griddata


robotData = np.loadtxt('/home/gonk/mren203_ws/src/mren203_package/scripts/contour/robot_data.txt')
x = robotData[:,0]
y = robotData[:,1]
z = robotData[:,2]

numPoints = 1000
xi,yi = np.meshgrid(np.linspace(x.min(), x.max(), numPoints),
                    np.linspace(y.min(), y.max(), numPoints))
zi = griddata((x, y), z, (xi, yi), method='linear')

fig = plt.figure(figsize=(8,6))
# Load the PGM image using the Pillow library
pgm_image = Image.open('/home/gonk/mren203_ws/src/mren203_package/scripts/contour/mymap.pgm')

grayscale_image = pgm_image.convert('L')
threshold = 128
bw_array = np.array(grayscale_image)
bw_array[bw_array < threshold] = 0  # set all pixel values below threshold to 0 (black)
bw_array[bw_array >= threshold] = 255  # set all pixel values above threshold to 255 (white)

bw_image = Image.fromarray(bw_array)
# Create the heat map
ax = fig.add_subplot(111) # for 2d

# Add the PGM image to the plot
pgm = ax.imshow(bw_image, cmap='gray',extent=[x.min()-3, x.max()+1, y.min()-1.0, y.max()+2.0])

heatmap = ax.imshow(zi,cmap='turbo',vmin=z.min(),vmax=z.max(),
                    extent=[x.min(),x.max(),y.min(),y.max()],
                    origin='lower', alpha = 0.70)

fig.colorbar(heatmap,orientation='vertical',shrink=0.75) #both
#fig.colorbar(heatmap) #2d
ax.scatter(x,y,color="black", alpha=0.5)
ax.set_aspect("equal", "box")

plt.show()
#plt.savefig('linux')
