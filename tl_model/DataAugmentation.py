import cv2
import csv
import math
import numpy as np
from random import randint
import matplotlib.pyplot as plt


class DataAugmentation:
    """ Provides basic methods to augment the data.

    Source: GitHub repository from Sven Bone
            https://github.com/SvenMuc/CarND-Behavioral-Cloning-P3
    """

    @staticmethod
    def prepare_dataset(csv_filename):
        """ Prepares the training and validation datasets (images and measurements) from driving log cvs file.
        
        :param csv_filename:              Path and filename of CVS file.
    
        :return: Returns the train_samples and validation_samples dataset.
        """

        # open and read content of csv file
        samples = []
        with open(csv_filename) as csv_file:
            reader = csv.reader(csv_file)

            # skip the csv header
            next(reader, None)

            for line in reader:
                samples.append(line)

        return samples

    @staticmethod
    def draw_overlay(image, frame=None, steering_angle=None, speed=None, color=(255, 255, 255)):
        """ Draws the image overlay like frame ID, speed, steering angle and the steering angle arrow.

        :param image:          Input image.
        :param frame:          Frame ID.
        :param steering_angle: Steering angle in degree.
        :param speed:          Speed in mph.
        :param color:          Text color as RGB array (R, G, B).

        :return: Returns the image with info text.
        """

        pos_x = 5
        pos_y = 15
        fontFace = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.4
        thickness = 1
        i = 0

        textSize = cv2.getTextSize('frame:', fontFace=fontFace, fontScale=fontScale, thickness=thickness)[0]

        if frame is not None:
            cv2.putText(img=image, text='frame:  {:05d}'.format(frame),
                        org=(pos_x, pos_y),
                        fontFace=fontFace, fontScale=fontScale, color=color, thickness=thickness)
            i += 1

        if speed is not None:
            cv2.putText(img=image, text='speed: {:6.2f} mph'.format(speed),
                        org=(pos_x, pos_y + i * (textSize[1] + 3)),
                        fontFace=fontFace, fontScale=fontScale, color=color, thickness=thickness)
            i += 1

        if steering_angle is not None:
            cv2.putText(img=image, text='angle: {:6.2f} deg'.format(steering_angle),
                        org=(pos_x, pos_y + i * (textSize[1] + 3)),
                        fontFace=fontFace, fontScale=fontScale, color=color, thickness=thickness)

            height = image.shape[0]
            width = image.shape[1]
            p0 = (int(width / 2), height)
            p1 = (int(width / 2) + int(height / 2 * math.tan(math.radians(steering_angle))), int(height / 2))
            cv2.line(image, p0, p1, color=(0, 255, 0), thickness=1, lineType=cv2.LINE_AA)

        return image

    @staticmethod
    def draw_steering_angles(image, steering_angle=None, augmented_steering_angle=None):
        """ Visualizes the steering angle by line in the center of the image.
        
        :param image:                     Input image.
        :param steering_angle:            Normed steering angle in (green)
        :param augmented_steering_angle:  Normed augmented steering angle (red).
        
        :return: Returns the image with visualized steering angle.
        """

        height = image.shape[0]
        width = image.shape[1]
        p0 = (int(width / 2), height)

        # draw steering angle
        if steering_angle is not None:
            p1 = (int(width / 2) + int(height / 2 * math.tan(math.radians(steering_angle * 25.))), int(height / 2))
            # TODO: p1 = (int(width / 2) + int(height * math.tan(math.radians(steering_angle * 25.))), 0)
            cv2.line(image, p0, p1, color=(0, 255, 0), thickness=1, lineType=cv2.LINE_AA)

        # draw augmented steering angle
        if augmented_steering_angle is not None:
            p1 = (int(width / 2) + int(height / 2 * math.tan(math.radians(augmented_steering_angle * 25.))), int(height / 2))
            # TODO: p1 = (int(width / 2) + int(height * math.tan(math.radians(augmented_steering_angle * 25.))), 0)
            cv2.line(image, p0, p1, color=(255, 0, 0), thickness=1, lineType=cv2.LINE_AA)

        return image

    @staticmethod
    def reduce_zero_steering_angles(samples, reduction_rate):
        """ Reduces total amount of images with 0° steering angle.
        
        :param samples:        Measurement samples which shall be augmented.
        :param reduction_rate: Reduction rate [0..1].
        :return: Returns the augmented samples array. If reduction rate is out of the range [0..1] the input samples
                 will be returned.
        """

        zero_angle_index = np.empty(0, dtype='int32')

        if reduction_rate < 1.:
            for idx, sample in enumerate(samples):
                if float(sample[3]) == 0.:
                    zero_angle_index = np.append(zero_angle_index, idx)

            nb_samples_to_delete = math.ceil(len(zero_angle_index) * reduction_rate)
            idx = np.arange(nb_samples_to_delete)
            np.random.shuffle(idx)
            return np.delete(samples, zero_angle_index[idx], 0)
        else:
            return samples

    @staticmethod
    def equalize_histogram(image):
        """ Equalizes the image histogram in order to improve the contrast.

        :param image:  Input RGB image.

        :return: Returns the equalized RGB image.
        """
        clahe = cv2.createCLAHE(clipLimit=0.95, tileGridSize=(8, 8))
        hls = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        hls[:, :, 0] = clahe.apply(hls[:, :, 0])
        hls[:, :, 1] = clahe.apply(hls[:, :, 1])
        hls[:, :, 2] = clahe.apply(hls[:, :, 2])

        return cv2.cvtColor(hls, cv2.COLOR_BGR2RGB)

    @staticmethod
    def flip_image_horizontally(image, image_gt, probability=1.0, lr_bias=0.0):
        """ Flip image horizontally with given 'probability'.
        
        :param image:           Input RGB image.
        :param image_gt:        Input ground truth image.
        :param probability:     Flip image probability [0..1].
        :param lr_bias:         Tendency to flip image more to the left or right 
                                [-1 = 100% left, 0 = equal +1 = 100% right].
        
        :return: Returns the flipped RGB and ground truth image.
        """

        if np.random.rand() <= probability:

            # 1 = flip to left, 2 = flip to right, (p_left + p_right = 1.0)
            p_right = (lr_bias + 1.) / 2.
            p_left = 1. - p_right
            lr_coin = np.random.choice([1, 2], p=[p_left, p_right])

            if lr_bias == 0.0 or (lr_coin == 1 and steering_angle < 0) or (lr_coin == 2 and steering_angle >= 0):
                flipped_image = cv2.flip(image, 1)
                flipped_image_gt = cv2.flip(image_gt, 1)

                return flipped_image, flipped_image_gt
            else:
                return image, image_gt
        else:
            return image, image_gt

    @staticmethod
    def random_brightness(image, probability=1.0):
        """ Augments the image by random brightness adjustment with given 'probability'.
        
        For HSV color space see: https://en.wikipedia.org/wiki/HSL_and_HSV
        
        :param image:          Input RGB image.
        :param probability:    Apply random brightness probability [0..1].
        
        :return:      Returns the brightness corrected RGB image.
        """

        if np.random.rand() <= probability:
            image_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            image_hsv = np.array(image_hsv, dtype=np.float64)
            rand_brightness = .2 + 0.5 * np.random.uniform()
            image_hsv[:, :, 2] = image_hsv[:, :, 2] * rand_brightness
            image_hsv[:, :, 2][image_hsv[:, :, 2] > 255] = 255
            image_hsv = np.array(image_hsv, dtype=np.uint8)
            image_rgb = cv2.cvtColor(image_hsv, cv2.COLOR_HSV2RGB)

            return image_rgb
        else:
            return image

    @staticmethod
    def random_blur(image, probability=1.0):
        """ Augments the image by random blur with given 'probability'.
        
        :param image:       Input RGB image.
        :param probability: Apply random blur probability [0..1].
        
        :return:      Returns the randomly blurred RGB image.
        """

        if np.random.rand() <= probability:
            return cv2.blur(image, (3, 3))
            # return cv2.medianBlur(image, 5)
            # return cv2.bilateralFilter(image, 9, 75, 75)
        else:
            return image

    @staticmethod
    def random_shadow(image, probability=1.0):
        """ Augments the image by random shadow areas with given 'probability'.

        :param image:       Input RGB image.
        :param probability: Apply random shadow probability [0..1].
        
        :return:      Returns the randomly shadow augmented RGB image.
        """

        if np.random.rand() <= probability:
            height = image.shape[0]
            width = image.shape[1]

            shadow_type = np.random.choice(['half', 'strip'])

            if shadow_type == 'half':
                # mask half of the image with shadow
                top_x, top_y = width * np.random.uniform(), height * np.random.uniform()
                bot_x, bot_y = width * np.random.uniform(), height * np.random.uniform()
                xm = np.mgrid[0:height, 0:width][1]
                ym = np.mgrid[0:height, 0:width][0]

                image_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
                image_hsv = np.array(image_hsv, dtype=np.float64)

                shadow_mask = 0 * image_hsv[:, :, 2]
                shadow_mask[((ym - top_y) * (bot_x - top_x) - (bot_y - top_y) * (xm - top_x) >= 0)] = 1

                random_bright = .2 + 0.5 * np.random.uniform()
                cond1 = shadow_mask == 1
                cond0 = shadow_mask == 0

                if np.random.randint(2) == 1:
                    image_hsv[:, :, 2][cond1] = image_hsv[:, :, 2][cond1] * random_bright
                else:
                    image_hsv[:, :, 2][cond0] = image_hsv[:, :, 2][cond0] * random_bright

                image_hsv[:, :, 2][image_hsv[:, :, 2] > 255] = 255
                image_hsv = np.array(image_hsv, dtype=np.uint8)
                return cv2.cvtColor(image_hsv, cv2.COLOR_HSV2RGB)
            elif shadow_type == 'strip':
                # add shadow strip
                shadow_image = np.copy(image)
                overlay = np.copy(image)

                direction = np.random.choice(['left', 'right', 'top', 'bottom'])

                if direction == 'left':
                    pt0 = [0, np.random.randint(0, 0.6 * height)]
                    pt1 = [np.random.randint(0.1 * width, 0.9 * width), np.random.randint(0, 0.8 * height)]
                    pt2 = [np.random.randint(0.1 * width, 0.9 * width), np.random.randint(min(pt1[1] * 1.2, height - 1), height)]
                    pt3 = [0, np.random.randint(min(pt0[1] * 1.5, height - 1), height)]
                elif direction == 'right':
                    pt0 = [width, np.random.randint(0, 0.6 * height)]
                    pt1 = [np.random.randint(0.1 * width, 0.9 * width), np.random.randint(0, 0.8 * height)]
                    pt2 = [np.random.randint(0.1 * width, 0.9 * width), np.random.randint(min(pt1[1] * 1.2, height - 1), height)]
                    pt3 = [width, np.random.randint(min(pt0[1] * 1.5, height - 1), height)]
                elif direction == 'top':
                    pt0 = [np.random.randint(0, 0.6 * width), 0]
                    pt1 = [np.random.randint(0, 0.8 * width), np.random.randint(0.5 * height, height)]
                    pt2 = [np.random.randint(min(pt1[0] * 1.2, width - 1), width), np.random.randint(0.5 * height, height)]
                    pt3 = [np.random.randint(min(pt0[0] * 1.5, width - 1), width), 0]
                elif direction == 'bottom':
                    pt0 = [np.random.randint(0, 0.6 * width), height]
                    pt1 = [np.random.randint(0, 0.8 * width), np.random.randint(0, 0.5 * height)]
                    pt2 = [np.random.randint(min(pt1[0] * 1.2, width - 1), width), np.random.randint(0, 0.5 * height)]
                    pt3 = [np.random.randint(min(pt0[0] * 1.5, width - 1), width), height]

                pts = np.array([pt0, pt1, pt2, pt3], np.int32)
                overlay = cv2.fillPoly(overlay, pts=[pts], color=(0, 0, 0))
                alpha = np.random.uniform(0.6, 0.85)
                return cv2.addWeighted(overlay, alpha, shadow_image, 1 - alpha, 0, shadow_image)
        else:
            return image

    @staticmethod
    def random_translation(image, image_gt, max_trans, probability=1.0, border_replication=True):
        """ Translates (shift) the image randomly horizontally and vertically with given 'probability'.

        :param image:              Input RGB image.
        :param image_gt:           Input ground truth image.
        :param max_trans:          Max translation in pixel [tx_max, ty_max].
        :param probability:        Apply random translation probability [0..1].
        :param border_replication: If true, the border will be replicated.
        
        :return: Returns the randomly translated RGB and ground truth image.
        """

        if np.random.rand() <= probability:
            height = image.shape[0]
            width = image.shape[1]
            tx = np.random.uniform(low=-max_trans[0], high=max_trans[0])
            ty = np.random.uniform(low=0, high=max_trans[1])
            M = np.float32([[1, 0, tx], [0, 1, ty]])

            if border_replication:
                t_image = cv2.warpAffine(image, M, (width, height), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
                t_image_gt = cv2.warpAffine(image_gt, M, (width, height), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
            else:
                t_image = cv2.warpAffine(image, M, (width, height))
                t_image_gt = cv2.warpAffine(image_gt, M, (width, height))

            return t_image, t_image_gt
        else:
            return image, image_gt

    @staticmethod
    def random_rotation(image, steering_angle, max_rotation, probability=1.0, border_replication=True):
        """ Rotates the image randomly around mid of bottom image row with given 'probability'.
        
        ATTENTION: This method shall only be applied, if the car is driving in the center of the road.
        
        :param image:              Input RGB image.
        :param steering_angle:     Input steering angle.
        :param max_rotation:       Max rotation angle in degree.
        :param probability:        Apply random rotation probability [0..1].
        :param border_replication: If true, the border will be replicated.
        
        :return: Returns the randomly rotated RGB image and the augmented steering angle.
        """

        if np.random.rand() <= probability:
            height = image.shape[0]
            width = image.shape[1]
            rot = np.random.uniform(low=-max_rotation, high=max_rotation)
            r_angle = steering_angle + 0.3 * (rot / 25.)
            M = cv2.getRotationMatrix2D((height, width / 2), rot, 1)

            if border_replication:
                r_image = cv2.warpAffine(image, M, (width, height), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
            else:
                r_image = cv2.warpAffine(image, M, (width, height))

            return r_image, r_angle
        else:
            return image, steering_angle

    @staticmethod
    def random_perspective_transformation(image, image_gt, max_trans, probability=1.0, border_replication=True):
        """ Applies random perspective transformation to image to simulate curves with given 'probability'.
        
        ATTENTION: This method shall only be applied, if the car is driving in the center of the road.
        
        :param image:              Input RGB image.
        :param image_gt:           Input ground truth image.
        :param max_trans:          Max transformation in pixel [tx_max, ty_max].
        :param probability:        Apply random perspective transformation probability [0..1].
        :param border_replication: If true, the border will be replicated.
        
        :return: Returns the randomly perspective transformed RGB and ground truth image.
        """

        if np.random.rand() <= probability:
            height = image.shape[0]
            width = image.shape[1]
            tx = np.random.uniform(low=-max_trans[0], high=max_trans[0])
            ty = np.random.uniform(low=-max_trans[1], high=max_trans[1])
            points1 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
            points2 = np.float32([[0+tx, 0+ty], [width+tx, 0+ty], [0, height], [width, height]])
            M = cv2.getPerspectiveTransform(points1, points2)

            if border_replication:
                t_image = cv2.warpPerspective(image, M, (width, height), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
                t_image_gt = cv2.warpPerspective(image_gt, M, (width, height), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
            else:
                t_image = cv2.warpPerspective(image, M, (width, height))
                t_image_gt = cv2.warpPerspective(image_gt, M, (width, height))

            return t_image, t_image_gt
        else:
            return image, image_gt

    @staticmethod
    def resize_image(image, shape):
        """ Resizes the image.

        :param image: Input RGB image.
        :param shape: New shape (width, height).

        :return: Returns the augmented RGB image and the augmented ground truth image.
        """
        return cv2.resize(image, shape, interpolation=cv2.INTER_AREA)

    @staticmethod
    def crop_image(image, roi, resize_size=None):
        """ Crops and resizes the image.
        
        :param image:       Input RGB image.
        :param roi:         Cropping area (region of interest) [x0, y0, x1, y1].
        :param resize_size: If not None the cropped image will be resized (width, height).
        
        :return: Returns the augmented RGB image and the augmented ground truth image.
        """

        cropped_image = image[roi[1]:roi[3], roi[0]:roi[2]]

        if resize_size is not None:
            return cv2.resize(cropped_image, resize_size, interpolation=cv2.INTER_AREA)
        else:
            return cropped_image


if __name__ == "__main__":

    print('Image augmentation:')
    print('Preparing training and validation datasets...', end='', flush=True)
    # samples = DataAugmentation.prepare_dataset('data/track_1_forwards.csv')
    # samples = DataAugmentation.prepare_dataset('data/track_2_special.csv')
    samples = DataAugmentation.prepare_dataset('data/track_1_fbrus_2_fbrs_ra0.9.csv')
    # samples = DataAugmentation.prepare_dataset('data/track_1_udacity.csv')
    print('done')

    #
    # Show random shadow images
    #
    # for i in range(len(samples)):
    #     idx = randint(0, len(samples))
    #     dataset_path = './data'
    #
    #     # load random images
    #     image = cv2.imread(dataset_path + '/' + samples[idx][0].lstrip())
    #     image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #     image = DataAugmentation.random_shadow(image)
    #     cv2.imshow('Shadow augmentation', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    #     cv2.waitKey(700)

    #
    # Show augmented images
    #
    print('Preparing figures...', end='', flush=True)
    nb_shown_images = 5
    nb_samples = len(samples)

    # blurred, equalized, brightness and shadow images
    fig1, axarr1 = plt.subplots(nb_shown_images, 5, figsize=(16, 9))
    plt.subplots_adjust(left=0.04, right=0.98, top=0.9, bottom=0.05, wspace=0.03, hspace=0.03)

    # flipped, rotated and transformed images
    fig2, axarr2 = plt.subplots(nb_shown_images, 5, figsize=(16, 9))
    plt.subplots_adjust(left=0.04, right=0.98, top=0.9, bottom=0.05, wspace=0.0, hspace=0.19)

    # cropped images
    fig3, axarr3 = plt.subplots(nb_shown_images, 5, figsize=(12, 9))
    plt.subplots_adjust(left=0.04, right=0.98, top=0.9, bottom=0.05, wspace=0.03, hspace=0.03)

    # center, left and right image
    fig4, axarr4 = plt.subplots(nb_shown_images, 3, figsize=(10, 9))
    plt.subplots_adjust(left=0.04, right=0.98, top=0.9, bottom=0.05, wspace=0.0, hspace=0.19)

    for i in range(nb_shown_images):
        idx = randint(0, nb_samples)
        dataset_path = './data'

        # load random images
        image = cv2.imread(dataset_path + '/' + samples[idx][0].lstrip())
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        steering_angle = float(samples[idx][3])

        image_left = cv2.imread(dataset_path + '/' + samples[idx][1].lstrip())
        image_left = cv2.cvtColor(image_left, cv2.COLOR_BGR2RGB)
        steering_angle_left = float(samples[idx][3]) + (6.25 / 25.)

        image_right = cv2.imread(dataset_path + '/' + samples[idx][2].lstrip())
        image_right = cv2.cvtColor(image_right, cv2.COLOR_BGR2RGB)
        steering_angle_right = float(samples[idx][3]) - (6.25 / 25.)

        # improve contrast
        equ_image = DataAugmentation.equalize_histogram(image)
        equ_image_left = DataAugmentation.equalize_histogram(image_left)
        equ_image_right = DataAugmentation.equalize_histogram(image_right)

        # augment images
        brightness_image = DataAugmentation.random_brightness(image)
        blurred_image = DataAugmentation.random_blur(image)
        shadow_image = DataAugmentation.random_shadow(image)
        flipped_image, flipped_steering_angle = DataAugmentation.flip_image_horizontally(image, steering_angle)
        translated_image, translated_steering_angle = DataAugmentation.random_translation(image, steering_angle, [30, 30])
        rotated_image, rotated_steering_angle = DataAugmentation.random_rotation(image, steering_angle, 10.)
        transformed_image, transformed_steering_angle = DataAugmentation.random_perspective_transformation(image, steering_angle, [200, 50])  # 40, 50

        # crop images
        width = image.shape[1]
        height = image.shape[0]
        roi = [20, 60, width - 20, height - 22]
        size = (64, 64)

        cropped_image = DataAugmentation.crop_image(image, roi, size)
        cropped_brightness_image = DataAugmentation.crop_image(brightness_image, roi, size)
        cropped_blurred_image = DataAugmentation.crop_image(blurred_image, roi, size)
        cropped_shadow_image = DataAugmentation.crop_image(shadow_image, roi, size)
        cropped_flipped_image = DataAugmentation.crop_image(flipped_image, roi, size)
        cropped_translated_image = DataAugmentation.crop_image(translated_image, roi, size)
        cropped_rotated_image = DataAugmentation.crop_image(rotated_image, roi, size)
        cropped_transformed_image = DataAugmentation.crop_image(transformed_image, roi, size)

        # draw steering angle direction into image
        image = DataAugmentation.draw_steering_angles(image, steering_angle=steering_angle)
        image_left = DataAugmentation.draw_steering_angles(image_left, steering_angle=steering_angle_left)
        image_right = DataAugmentation.draw_steering_angles(image_right, steering_angle=steering_angle_right)
        flipped_image = DataAugmentation.draw_steering_angles(flipped_image, augmented_steering_angle=flipped_steering_angle)
        translated_image = DataAugmentation.draw_steering_angles(translated_image, steering_angle=steering_angle, augmented_steering_angle=translated_steering_angle)
        rotated_image = DataAugmentation.draw_steering_angles(rotated_image, steering_angle=steering_angle, augmented_steering_angle=rotated_steering_angle)
        transformed_image = DataAugmentation.draw_steering_angles(transformed_image, steering_angle=steering_angle, augmented_steering_angle=transformed_steering_angle)

        cropped_image = DataAugmentation.draw_steering_angles(cropped_image, steering_angle=steering_angle)
        cropped_flipped_image = DataAugmentation.draw_steering_angles(cropped_flipped_image, augmented_steering_angle=flipped_steering_angle)
        cropped_translated_image = DataAugmentation.draw_steering_angles(cropped_translated_image, steering_angle=steering_angle, augmented_steering_angle=translated_steering_angle)
        cropped_rotated_image = DataAugmentation.draw_steering_angles(cropped_rotated_image, steering_angle=steering_angle, augmented_steering_angle=rotated_steering_angle)
        cropped_transformed_image = DataAugmentation.draw_steering_angles(cropped_transformed_image, steering_angle=steering_angle, augmented_steering_angle=transformed_steering_angle)

        # show blurred, equalized, brightness and shadow images
        axarr1[i, 0].imshow(image)
        axarr1[i, 0].axis('off')
        axarr1[i, 1].imshow(equ_image)
        axarr1[i, 1].axis('off')
        axarr1[i, 2].imshow(brightness_image)
        axarr1[i, 2].axis('off')
        axarr1[i, 3].imshow(blurred_image)
        axarr1[i, 3].axis('off')
        axarr1[i, 4].imshow(shadow_image)
        axarr1[i, 4].axis('off')

        # show flipped, rotated and transformed images
        axarr2[i, 0].imshow(image)
        axarr2[i, 0].set_title('orginial {:.4f}'.format(steering_angle * 25.))
        axarr2[i, 0].axis('off')
        axarr2[i, 1].imshow(flipped_image)
        axarr2[i, 1].set_title('flipped {:.4f}->{:.4f}'.format(steering_angle * 25., flipped_steering_angle * 25.))
        axarr2[i, 1].axis('off')
        axarr2[i, 2].imshow(translated_image)
        axarr2[i, 2].set_title('translated {:.4f}->{:.4f}'.format(steering_angle * 25., translated_steering_angle * 25.))
        axarr2[i, 2].axis('off')
        axarr2[i, 3].imshow(rotated_image)
        axarr2[i, 3].set_title('rotated {:.4f}->{:.4f}'.format(steering_angle * 25., rotated_steering_angle * 25.))
        axarr2[i, 3].axis('off')
        axarr2[i, 4].imshow(transformed_image)
        axarr2[i, 4].set_title('transformed {:.4f}->{:.4f}'.format(steering_angle * 25., transformed_steering_angle * 25.))
        axarr2[i, 4].axis('off')

        # show cropped images
        axarr3[i, 0].imshow(cropped_image)
        axarr3[i, 0].axis('off')
        axarr3[i, 1].imshow(cropped_flipped_image)
        axarr3[i, 1].axis('off')
        axarr3[i, 2].imshow(cropped_translated_image)
        axarr3[i, 2].axis('off')
        axarr3[i, 3].imshow(cropped_rotated_image)
        axarr3[i, 3].axis('off')
        axarr3[i, 4].imshow(cropped_transformed_image)
        axarr3[i, 4].axis('off')

        # show center, left and right images
        axarr4[i, 0].imshow(image_left)
        axarr4[i, 0].set_title('left {:.4f}'.format(steering_angle_left * 25.))
        axarr4[i, 0].axis('off')
        axarr4[i, 1].imshow(image)
        axarr4[i, 1].set_title('center {:.4f}'.format(steering_angle * 25.))
        axarr4[i, 1].axis('off')
        axarr4[i, 2].imshow(image_right)
        axarr4[i, 2].set_title('right {:.4f}'.format(steering_angle_right * 25.))
        axarr4[i, 2].axis('off')

    # set titles blurred, brightness and shadow images
    axarr1[0, 0].set_title('original')
    axarr1[0, 1].set_title('hist. equalized')
    axarr1[0, 2].set_title('brightness')
    axarr1[0, 3].set_title('blurred')
    axarr1[0, 4].set_title('shadow')

    # set titles cropped images
    axarr3[0, 0].set_title('cropped original')
    axarr3[0, 1].set_title('cropped flipped')
    axarr3[0, 2].set_title('cropped translation')
    axarr3[0, 3].set_title('cropped rotation')
    axarr3[0, 4].set_title('cropped transformation')

    title = 'Image Augmentation (w={:d}, h={:d})'.format(image.shape[1], image.shape[0])
    fig1.suptitle(title)

    title = 'Image Augmentation (w={:d}, h={:d})'.format(image.shape[1], image.shape[0])
    fig2.suptitle(title)

    title = 'Cropped Images (w={:d}, h={:d})'.format(cropped_image.shape[1], cropped_image.shape[0])
    fig3.suptitle(title)

    title = 'Left, Center and Right Image (w={:d}, h={:d})'.format(image.shape[1], image.shape[0])
    fig4.suptitle(title)

    print('done')
    print('Close the figures to continue...')
    plt.show()
