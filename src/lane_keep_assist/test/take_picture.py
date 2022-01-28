import numpy as np
import airsim
import os

# Just a fn to take to a picture and save it while using airsim
def take_picture(filename):
    client = airsim.CarClient()
    client.confirmConnection()

    responses = client.simGetImages([airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])
    response = responses[0]

    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)
    airsim.write_png(os.path.normpath(filename + '.png'), img_rgb)
