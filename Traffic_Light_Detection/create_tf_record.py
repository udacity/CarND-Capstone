import tensorflow as tf
import os
import io
import PIL.Image
from lxml import etree
from object_detection.utils import dataset_util, label_map_util
from tqdm import tqdm
from sklearn.model_selection import train_test_split

flags = tf.app.flags

flags.DEFINE_string('data_dir', None, 'Path to the folder where the images are stored')
flags.DEFINE_string('labels_dir', None, 'Path to the folder labels annotation are stored')
flags.DEFINE_string('labels_map_path', None, 'Path to the labels map pbtxt file')
flags.DEFINE_string('output_path', None, 'Path to output record file, if split_train_test is enabled creates two file one for training and one for validation')
flags.DEFINE_float('split_train_test', 0.25, 'If supplied specifies the amount of samples to use for evaluation')

tf.app.flags.mark_flag_as_required('data_dir')
tf.app.flags.mark_flag_as_required('labels_map_path')
tf.app.flags.mark_flag_as_required('output_path')

FLAGS = flags.FLAGS

def create_tf_example(data_dir, xml_dict, labels_map):
    data = xml_dict['annotation']
    file_path = os.path.join(data_dir, data['filename'])
    with tf.gfile.GFile(file_path, 'rb') as fid:
        encoded_jpg = fid.read()
    encoded_jpg_io = io.BytesIO(encoded_jpg)
    image = PIL.Image.open(encoded_jpg_io)
    if image.format != 'JPEG':
        raise ValueError('Image format not JPG')

    xmin = []
    ymin = []
    xmax = []
    ymax = []
    classes = []
    classes_text = []
    truncated = []
    
    width = int(data['size']['width'])
    height = int(data['size']['height'])
    filename = data['filename'].encode('utf8')

    for obj in data['object']:
        xmin.append(float(obj['bndbox']['xmin']) / width)
        ymin.append(float(obj['bndbox']['ymin']) / height)
        xmax.append(float(obj['bndbox']['xmax']) / width)
        ymax.append(float(obj['bndbox']['ymax']) / height)
        classes_text.append(obj['name'].encode('utf8'))
        classes.append(labels_map[obj['name']])
        truncated.append(int(obj['truncated']))

    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(filename),
        'image/source_id': dataset_util.bytes_feature(filename),
        'image/encoded': dataset_util.bytes_feature(encoded_jpg),
        'image/format': dataset_util.bytes_feature(r'jpg'.encode('utf8')),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmin),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmax),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymin),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymax),
        'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
        'image/object/class/label': dataset_util.int64_list_feature(classes),
        'image/object/truncated': dataset_util.int64_list_feature(truncated)
    }))
    return tf_example

def create_tf_record(label_files, data_dir, labels_map, output_path):
    
    writer = tf.python_io.TFRecordWriter(output_path)
    
    for label_file in tqdm(label_files, desc='Converting', unit=' images'):
        with tf.gfile.GFile(label_file, 'r') as f:
            xml = f.read()
        xml = etree.fromstring(xml)
        xml_dict = dataset_util.recursive_parse_xml_to_dict(xml)
        tf_record = create_tf_example(data_dir, xml_dict, labels_map)
        writer.write(tf_record.SerializeToString())
    
    writer.close()

def main(unused_argv):

    data_dir = FLAGS.data_dir
    if FLAGS.labels_dir is None:
        FLAGS.labels_dir = os.path.join(data_dir, 'labels')
    labels_map = label_map_util.get_label_map_dict(FLAGS.labels_map_path)
    label_files_train = os.listdir(FLAGS.labels_dir)
    label_files_train = [os.path.join(FLAGS.labels_dir, file_name) for file_name in label_files_train]
    output_path_train = FLAGS.output_path
    split_train_test = FLAGS.split_train_test

    print('Total samples: {}'.format(len(label_files_train)))
    
    if split_train_test:
        label_files_train, label_files_eval = train_test_split(label_files_train, test_size = split_train_test, shuffle = True)
        dir_path = os.path.dirname(output_path_train)
        
        if len(dir_path) and not os.path.isdir(dir_path):
            os.makedirs(dir_path)

        file_name_split = os.path.splitext(os.path.basename(output_path_train))
        
        if file_name_split[1] == '':
            file_name_split = (file_name_split[0], '.record')

        output_path_train = os.path.join(dir_path, '{}_train{}'.format(file_name_split[0], file_name_split[1]))
        output_path_eval = os.path.join(dir_path, '{}_eval{}'.format(file_name_split[0], file_name_split[1]))
    
    create_tf_record(label_files_train, data_dir, labels_map, output_path_train)
    print('TF record file for training created with {} samples: {}'.format(len(label_files_train), output_path_train))
    
    if label_files_eval:
        create_tf_record(label_files_eval, data_dir, labels_map, output_path_eval)
        print('TF record file for validation created with {} samples: {}'.format(len(label_files_eval), output_path_eval))

if __name__ == '__main__':
  tf.app.run()