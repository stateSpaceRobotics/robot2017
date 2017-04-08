import math


class LaserObject(object):
    """
    Data structure used to hold objects detected in laser scans
    """
    def __init__(self):
        self.left_edge = None  # index of left edge
        self.right_edge = None  # index of right edge
        self.length = None  # obj length
        self.centroid = None  # index of object centroid
        self.distance = None  # centroid range in scan
        self.angle = None  # centroid angle in scan

    def process(self, scan_msg):
        """
        Given scan_msg and left and right edges, calculate:
         - length
         - centroid
         - distance
         - angle
        """
        assert self.left_edge is not None and self.right_edge is not None, "Object edges not set.  Cannot process."
        arc_ang = (self.left_edge - self.right_edge) * scan_msg.angle_increment
        right_dist = scan_msg.ranges[self.right_edge]
        left_dist = scan_msg.ranges[self.left_edge]

        try:
            self.length = math.sqrt((right_dist**2 + left_dist**2) - (2 * right_dist * left_dist * math.cos(arc_ang)))
        except:
            self.length = 0

        self.centroid = int(abs(self.right_edge - self.left_edge) / 2) + self.right_edge

        try:
            self.distance = scan_msg.ranges[self.centroid]
        except:
            self.distance = 0

        self.angle = self.centroid * scan_msg.angle_increment
