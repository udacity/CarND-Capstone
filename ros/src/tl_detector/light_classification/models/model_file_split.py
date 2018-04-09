'''
Simple file splitter to break up traffic light classifier model file, based on:
https://github.com/csmunuku/file_splitter_joiner
'''


def split_file(input_file, chunk_size=49000000):
    in_file = open(input_file, 'rb')
    data = in_file.read()
    in_file.close()

    data_bytes = len(data)
    chunk_filenames = []
    split_idx = 0
    for i in range(0, data_bytes + 1, chunk_size):
        split_idx += 1
        t_filename = '{}.{:03}'.format(input_file, split_idx)
        chunk_filenames.append(t_filename)
        out_file = open(t_filename, 'wb')
        out_file.write(data[i:i + chunk_size])
        out_file.close


if __name__ == '__main__':
    # Split model file into 4x 49 MB chunks
    model_file = 'ResNet50-UdacityRealandSimMix-Best-val_acc-1.0.hdf5'
    split_file(model_file)
