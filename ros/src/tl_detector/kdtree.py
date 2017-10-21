#!/usr/bin/env python

# kd-tree index and nearest neighbour search
# includes doctests, run with: python -m doctest kdtree.py

# Originally from: https://gist.github.com/tompaton/863301
# [brahm] Made some modifications to be more self contained and 
#         fix depth/dimension issue

class KDTree(object):
    """
    kd-tree spatial index and nearest neighbour search
    http://en.wikipedia.org/wiki/Kd-tree
    """
    
    def __init__(self, point_list, _depth=0):
        """
        Initialize kd-tree index with points.

        >>> KDTree([])
        None
        >>> KDTree([(1,1)])
        (0, (1, 1), None, None)
        >>> KDTree([(1,1),(2,2)])
        (0, (2, 2), (1, (1, 1), None, None), None)
        >>> KDTree([(1,1),(2,2),(3,3)])
        (0, (2, 2), (1, (1, 1), None, None), (1, (3, 3), None, None))
        """
        if point_list:
            # Select axis based on depth so that axis cycles through all valid values
            # [brahm] Note: adjusted to only use 2d points so that 3rd 
            # position in tuple can be used to track original index in 
            # point list. Also, _distance() function currently only measures 2d.
            #self.axis = _depth % len(point_list[0]) 
            self.axis = _depth % 2
                    
            # Sort point list and choose median as pivot element
            point_list = sorted(point_list, key=lambda point: point[self.axis])
            median = len(point_list) // 2 # choose median

            # Create node and construct subtrees
            self.location = point_list[median]
            self.child_left = KDTree(point_list[:median], _depth + 1)
            self.child_right = KDTree(point_list[median + 1:], _depth + 1)
        else:
            self.axis = 0
            self.location = None
            self.child_left = None
            self.child_right = None

    def closest_point(self, point, _best=None):
        """
        Efficient recursive search for nearest neighbour to point

        >>> t = KDTree([(2,3), (5,4), (9,6), (4,7), (8,1), (7,2)])
        >>> t
        (0, (7, 2), (1, (5, 4), (0, (2, 3), None, None), (0, (4, 7), None, None)), (1, (9, 6), (0, (8, 1), None, None), None))
        >>> t.closest_point( (7,2) )
        (7, 2)
        >>> t.closest_point( (8,1) )
        (8, 1)
        >>> t.closest_point( (1,1) )
        (2, 3)
        >>> t.closest_point( (5,5) )
        (5, 4)
        """
        if self.location is None:
            return _best
 
        if _best is None:
            _best = self.location
 
        # consider the current node
        if self._distance(self.location, point) < self._distance(_best, point):
            _best = self.location
 
        # search the near branch
        _best = self._child_near(point).closest_point(point, _best)
 
        # search the away branch - maybe
        if self._distance_axis(point) < self._distance(_best, point):
            _best = self._child_away(point).closest_point(point, _best)
 
        return _best

    # internal methods
    
    def __repr__(self):
        """
        Simple representation for doctests
        """
        if self.location:
            return "(%d, %s, %s, %s)" % (self.axis, repr(self.location), repr(self.child_left), repr(self.child_right))
        else:
            return "None"

    def _distance_axis(self, point):
        """
        Squared distance from current node axis to point

        >>> KDTree([(1,1)])._distance_axis((2,3))
        1
        >>> KDTree([(1,1),(2,2)]).child_left._distance_axis((2,3))
        4
        """
        # project point onto node axis
        # i.e. want to measure distance on axis orthogonal to current node's axis
        axis_point = list(point)
        axis_point[self.axis] = self.location[self.axis]
        return self._distance(tuple(axis_point), point)

    def _child_near(self, point):
        """
        Either left or right child, whichever is closest to the point
        """
        if point[self.axis] < self.location[self.axis]:
            return self.child_left
        else:
            return self.child_right

    def _child_away(self, point):
        """
        Either left or right child, whichever is furthest from the point
        """
        if self._child_near(point) is self.child_left:
            return self.child_right
        else:
            return self.child_left  
  
    # helper function
    # [brahm] Changed this to a member function to make sure it doesn't
    # conflict with other functions in the global namespace.
    def _distance(self, a, b):
        """
        Squared distance between points a & b
        """
        return (a[0]-b[0])**2 + (a[1]-b[1])**2
