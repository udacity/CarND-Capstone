#!/usr/bin/env python
import os
import sys
import argparse
import yaml

import matplotlib.pyplot as plt
import numpy as np

import plot_utils as plu


class DatasetHandler():
    """
    The DatasetHandler provides basic methods to translate the different dataset ground truth data into
    one common format which can be used to train the TL model.

    Usage:

    """

    bosch_label_dict_valid = False      # True if Bosch dataset has been read.
    bosch_label_dict = {}               # dictionary with Bosch label data
    bosch_label_statistics = {}         # dictionary of type <TL class> : <number of labeled TL class>

    def read_all_bosch_labels(self, input_yaml, riib=False):
        """
        Gets all labels from the Bosch Small Traffic Light Dataset listed in the label file.

        Image Formats:
          - RIIB 12 bit (red-clear-clear-blue), size = 1280x736
          - RGB 8bit (red-green-blue), size = 1280x720, Reconstructed color images.

        The RGB images are provided for debugging and can also be used for training. However, the RGB conversion
        process has some drawbacks. Some of the converted images may contain artifacts and the color distribution
        may seem unusual.

        :param input_yaml: Path to yaml file.
        :param riib:       If True, change path to labeled pictures.

        :return: images:   Labels for traffic lights.
        """
        images = yaml.load(open(input_yaml, 'rb').read())

        for i in range(len(images)):
            images[i]['path'] = os.path.abspath(os.path.join(os.path.dirname(input_yaml), images[i]['path']))

            if riib:
                images[i]['path'] = images[i]['path'].replace('.png', '.pgm')
                images[i]['path'] = images[i]['path'].replace('rgb/train', 'riib/train')
                images[i]['path'] = images[i]['path'].replace('rgb/test', 'riib/test')
                for box in images[i]['boxes']:
                    box['y_max'] = box['y_max'] + 8
                    box['y_min'] = box['y_min'] + 8

            # generate label statistics
            for box in images[i]['boxes']:
                if box['label'] not in self.bosch_label_statistics.keys():
                    self.bosch_label_statistics[box['label']] = 1
                else:
                    self.bosch_label_statistics[box['label']] = self.bosch_label_statistics[box['label']] + 1

        self.bosch_label_dict = images
        self.bosch_label_dict_valid = True

        return images

    def plot_bosch_label_histogram(self, safe_figure=False):
        """ Plots a histogram over all Bosch labels which have been read by the `read_all_bosch_labels()` method.

        :param safe_figure:  If true safe figure as png file.
        """

        if not self.bosch_label_dict_valid:
            print('ERROR: No valid dataset dictionary. Read Bosch dataset before.')
            return

        x = np.arange(len(self.bosch_label_statistics))
        label_names = np.array([], dtype=np.str)
        label_hist = np.array([])
        colors = []

        for key in sorted(self.bosch_label_statistics.keys()):
            print('{} = {}'.format(key, self.bosch_label_statistics[key]))
            label_names = np.append(label_names, key)
            label_hist = np.append(label_hist, self.bosch_label_statistics[key])

            # set bar color depending on label class
            if str(key).lower().find('green') >= 0:
                colors.append(plu.COLOR_GREEN)
            elif str(key).lower().find('yellow') >= 0:
                colors.append(plu.COLOR_YELLOW)
            elif str(key).lower().find('red') >= 0:
                colors.append(plu.COLOR_RED)
            else:
                colors.append(plu.COLOR_GREY)

        # plot label histogram
        fig, ax = plt.subplots()
        fig.subplots_adjust(left=0.27, right=0.97, bottom=0.1, top=0.9)
        rect = plt.barh(x, label_hist, color=colors)
        plu.autolabel_barh(rect, ax)
        plt.yticks(x, label_names)
        plt.title('Label Distribution in Bosch Small Traffic Light Dataset')
        plt.ylabel('label class')
        plt.xlabel('number of labels per class')
        ax.set_axisbelow(True)
        #ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #ax.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')

        if safe_figure:
            plt.savefig('bosch_label_histogram.png')

        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dataset Handler provides statistics and label transformation.')

    parser.add_argument(
        '--bosch_label_file',
        help='Path to the Bosch label YAML file.',
        dest='bosch_label_file',
        metavar='YAML_file'
    )

    parser.add_argument(
        '-s', '--safe_plots',
        help='Safe plots as PNG files.',
        action='store_true',
        default=False,
        dest='safe_plots'
    )

    args = parser.parse_args()

    if len(sys.argv) == 1:
        # no arguments found
        parser.print_usage()
        parser.exit(-1)

    safe_plots = False

    if args.bosch_label_file:
        print('Analyse Bosch dataset...')
        dataset_handler = DatasetHandler()
        dataset_handler.read_all_bosch_labels(args.bosch_label_file)

        if args.safe_plots:
            safe_plots = True

        dataset_handler.plot_bosch_label_histogram(safe_figure=safe_plots)
