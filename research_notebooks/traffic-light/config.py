import os

# change me! absolute path to store datasets
# download_dir = 'D:/lots/of/data'
download_dir = './data'

def set_download_path(dst):
    print('Downloading to ', os.path.abspath(dst))
    download_dir = dst

datasets = [
    'https://s3-us-west-2.amazonaws.com/traffic-light-data/just-traffic-lights.zip',
]
