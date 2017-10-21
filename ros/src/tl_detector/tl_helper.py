"""
Helper and utils function to improve code readability in `tl_detector.py`
"""


import os
from os.path import exists


def create_dir_if_nonexistent(dir_path):
    """
    Check if a directory exists and create it if not.
    """
    if not exists(dir_path):
        os.makedirs(dir_path)
    return dir_path