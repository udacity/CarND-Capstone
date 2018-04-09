'''
Simple file joiner to recombine traffic light classifier model file, based on:
https://github.com/csmunuku/file_splitter_joiner
'''


def join_file(base_filename, num_chunks):
    data_combined = []
    split_idx = 0
    for i in range(0, num_chunks, 1):
        split_idx += 1
        t_filename = '{}.{:03}'.format(base_filename, split_idx)
        in_file = open(t_filename, 'rb')
        data_combined.append(in_file.read())
        in_file.close()

    out_file = open(base_filename, 'wb')
    for data in data_combined:
        out_file.write(data)
    out_file.close()


if __name__ == '__main__':
    # Join model file from 4x 49 Mb chunks
    model_file = 'ResNet50-UdacityRealandSimMix-Best-val_acc-1.0.hdf5'
    join_file(model_file, 4)
