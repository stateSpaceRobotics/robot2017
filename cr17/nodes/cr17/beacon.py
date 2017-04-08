class Beacon(object):
    """
    Data structure used to store information on beacon found in laser scan
    """
    def __init__(self, right_post, left_post, actual_dist, err):
        self.right_post = right_post    # LaserObject that stores right post
        self.left_post = left_post     # LaserObject that stores left post
        self.actual_dist = actual_dist  # Actual distance between right and left post
        self.err = err                  # (POST_DIST) - |DIST(right_post, left_post)|
