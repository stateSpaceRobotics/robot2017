#!/usr/bin/env python
PKG='cr17'
NAME='test_navigator'

import unittest
import rostest

from cr17 import navigator


class TestNavigator(unittest.TestCase):

    def test_one_equals_one(self):
        self.assertEquals(1, 1)

if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestNavigator)
