import json
import pickle
from tkinter import filedialog as fd
from Models import RoadSegment, RoadSegmentType, Lane, MapModel
from typing import Dict, List


# Responsible for saving/loading a road_segments dict to/from a json file
def save_to_file(map_model: MapModel):
    filename = fd.askopenfilename(initialdir='.')
    outfile = open(filename, 'wb')

    pickle.dump(map_model, outfile)
    outfile.close()


# load from save file
def load_from_file() -> MapModel:
    filename = fd.askopenfilename(initialdir='.')
    infile = open(filename, 'rb')
    map_model: MapModel = pickle.load(infile)

    # Warning message for version mismatch
    if map_model.instance_version != MapModel.version:
        print("\033[1m"f'WARNING: version mismatch between MapModel class version ({MapModel.version}) '
              f'and serialized version ({map_model.instance_version})'"\033[0m")

    infile.close()
    return map_model
