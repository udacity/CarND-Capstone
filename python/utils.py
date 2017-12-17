import os
import numpy as np
from os.path import isfile, join

def files_only(dir):
    #list = [join(dir, f) for f in os.listdir(dir) if isfile(join(dir, f))]
    list = []

    for f in os.listdir(dir):
        path = join(dir, f)

        if (isfile(path)):
            list.append(path)

    return list

def files_from(init_dir, include_sub_directories = False):
    if not init_dir:
        return []

    if include_sub_directories:
        dirs = [x[0] for x in os.walk(init_dir)]
    else:
        dirs = [init_dir]

    files = []

    for dir in dirs:
        files.extend(files_only(dir))

    return files
