import numpy as np
import airsim
import os


# Just a fn to take to a picture and save it while using airsim
def take_picture(index):
    client = airsim.CarClient()
    client.confirmConnection()

    responses = client.simGetImages([airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])
    response = responses[0]

    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)
    airsim.write_png(os.path.normpath('o_' + index + '.png'), img_rgb)
    airsim.write_png(os.path.normpath('a_' + index + '.png'), img_rgb)

    responses = client.simGetImages([airsim.ImageRequest("1", airsim.ImageType.Segmentation, False, False)])
    response = responses[0]

    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)
    airsim.write_png(os.path.normpath('s_' + index + '.png'), img_rgb)
