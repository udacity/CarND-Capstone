#!/usr/bin/env python
"""
The plot utils provide generic functions and enhancements for the matplotlib library.
"""

""" Color specifications """
COLOR_GREEN  = '#28B463'
COLOR_RED    = '#E74C3C'
COLOR_YELLOW = '#F4D03F'
COLOR_GREY   = '#A6ACAF'


def autolabel_bar(rects, ax):
    """
    Attach a text label above each vertical bar displaying its height

    :param rects Handle to bar rectangles.
    :param ax    Handle to axis.
    """
    for rect in rects:
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width()/2., 1.05*height, '%d' % int(height), ha='center', va='bottom')


def autolabel_barh(rects, ax):
    """
    Attach a text label next to each horizontal bar displaying its height

    :param rects Handle to bar rectangles.
    :param ax    Handle to axis.
    """
    for rect in rects:
        width = rect.get_width()
        height = rect.get_height()
        ax.text(rect.get_x() + width + 1. , rect.get_y() + height/2., '%d' % int(width), ha='left', va='center')