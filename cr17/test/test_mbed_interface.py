#!/usr/bin/env python

PKG = 'cr17'
NAME = 'test_mbed_interface'

import unittest
import rostest

from cr17 import mbed_interface


class TestMbedInterface(unittest.TestCase):

	####Test methods###

	#Test for fixed_to_float_2

	#Test for float_to_fixed_2

	#Test for fixed_to_float

	#Test for float_to_fixed


if __name__ == '__main__':
	rostest.unitrun(PKG, NAME, TestMbedInterface)

