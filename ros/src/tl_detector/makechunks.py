# !/usr/bin/env python
import os
import sys
from functools import partial

megas = 5
model_path = '../trained_model/frozen_inference_graph.pb'
chunks_folder = 'frozen_model_chunks'


# check that we run python 2.7
assert sys.version_info < (3, 0)

args = sys.argv

if args[1]:
    model_path = args[1]
if args[2]:
    chunks_folder = args[2]
if args[3]:
    megas = int(args[3])


if not os.path.exists(chunks_folder):
    os.mkdir(chunks_folder)
else:
    for fname in os.listdir(chunks_folder):
        os.remove(os.path.join(chunks_folder, fname))
chunknum = 0
chunksize = 1024 * 1024

print("Splitting " + model_path + " into folder " + chunks_folder + " in pieces of " + str(megas) + " MB")

with open(model_path, 'rb') as infile:
    for chunk in iter(partial(infile.read, chunksize * megas), ''):
        print('Creating chunk %04d' % (chunknum))
        ofilename = os.path.join(chunks_folder, ('chunk%04d' % (chunknum)))
        outfile = open(ofilename, 'wb')
        outfile.write(chunk)
        outfile.close()
        chunknum += 1