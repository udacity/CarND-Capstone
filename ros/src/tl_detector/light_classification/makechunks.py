# !/usr/bin/env python
import os
import sys
from functools import partial

curr_dir = os.path.dirname(os.path.realpath(__file__))

megas = 5
sim = True
model_path = curr_dir + '/../../trained_model/frozen_inference_graph.pb'
sim_arg = 'sim'



# check that we run python 2.7
assert sys.version_info < (3, 0)

args = sys.argv



if len(args) > 1:
    model_path = args[1]
if len(args) > 2:
    sim_arg = args[2]
if len(args) > 3:
    megas = int(args[3])


sim = sim_arg != 'real'

if sim:
    model_folder = '/sim_model'
else:
    model_folder = '/real_model'

chunks_folder = curr_dir + model_folder + '/chunks'


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