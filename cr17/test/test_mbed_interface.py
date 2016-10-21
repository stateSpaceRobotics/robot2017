#!/usr/bin/env python

PKG = 'cr17'
NAME = 'test_mbed_interface'

import unittest
import rostest

from cr17 import mbed_interface

SINGLE_BYTE_UPPER_LIMIT = 7.9
SINGLE_BYTE_LOWER_LIMIT = -7.9


class TestMbedInterface(unittest.TestCase):

	####Test methods###

	#Test for fixed_to_float_2

	#Test for float_to_fixed_2

	#Test for fixed_to_float

	#Test for float_to_fixed
	def test_upper_limit_rejection(self):
		self.assertRaises(ValueError, mbed_interface.float_to_fixed, 8.0)



if __name__ == '__main__':
	rostest.unitrun(PKG, NAME, TestMbedInterface)

