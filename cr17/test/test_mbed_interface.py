#!/usr/bin/env python

PKG = 'cr17'
NAME = 'test_mbed_interface'

import unittest
import rostest

from cr17 import mbed_interface

SINGLE_BYTE_UPPER_LIMIT = 7.9
SINGLE_BYTE_LOWER_LIMIT = -7.9


class TestMbedInterface(unittest.TestCase):

######Test methods######
    
    #=============================#
    #  Test for fixed_to_float_2  #
    #=============================#
    #Makes sure it rejects non int values
    def test_fixed_only_input(self):
        self.assertRaises(TypeError, mbed_interface.fixed_to_float_2(1.0, 1.0))
        self.assertRaises(TypeError, mbed_interface.fixed_to_float_2(100, 1.0))
        self.assertRaises(TypeError, mbed_interface.fixed_to_float_2(1.0, 100))


    #Makes sure values greater than the limits are rejected
    def test_fixed_input_upper_limit(self):
        self.assertRaises(ValueError, mbed_interface.fixed_to_float_2, 256, 31)
        self.assertRaises(ValueError, mbed_interface.fixed_to_float_2, 255, 32)
        self.assertRaises(ValueError, mbed_interface.fixed_to_float_2, 256, 32)

    #Makes sure values less than zero are rejected
    def test_fixed_input_lower_limit(self):
        self.assertRaises(ValueError, mbed_interface.fixed_to_float_2, -140,  5)
        self.assertRaises(ValueError, mbed_interface.fixed_to_float_2,  140, -5)
        self.assertRaises(ValueError, mbed_interface.fixed_to_float_2, -140, -5)

    #Test different input output pairs
    def test_fixed_input_float_output_pairs_2(self):
      input_output_pairs = (
                           (0,0,0.0),
                           (0,10,0.625),
                           (0,31,1.9375),
                           (100,0,200.0),
                           (100,31, 201.9375),
                           (255, 31, 511.9375)
                           )
      for MSB, LSB, output_float in input_output_pairs:
          self.assertEqual(mbed_interface.fixed_to_float_2(MSB,LSB), output_float)


    #=============================#
    #  Test for float_to_fixed_2  #
    #=============================#

    def test_float_input_2(self):
        self.assertRaises(TypeError, mbed_interface.float_to_fixed_2, 100)

    def test_float_input_upper_limit(self):
        self.assertRaises(ValueError, mbed_interface.float_to_fixed_2, 512.0)

    def test_float_input_lower_limit(self):
        self.assertRaises(ValueError, mbed_interface.float_to_fixed_2, -1.0)

    def test_float_input_fixed_output_pairs_2(self):
        input_output_pairs = (
                             (0.0, 0,0),
                             (0.625, 0, 10),
                             (1.9375, 0, 31),
                             (200.0, 100, 0),
                             (201.9375, 100, 31),
                             (511.9, 255, 31)
                             )
        for float_val, MSB, LSB in input_output_pairs:
            return_MSB, return_LSB = mbed_interface.float_to_fixed_2(float_val)
            self.assertAlmostEqual(return_LSB, LSB, delta = 1)
            self.assertAlmostEqual(return_MSB, MSB, delta = 1)

    #===========================#
    #  Test for fixed_to_float  #
    #===========================#

    #Test for TypeError if an fixed(int) is not passed in
    def test_fixed_only_input(self):
        self.assertRaises(TypeError, mbed_interface.fixed_to_float, 1.0)

    #Test if value error is thrown for being greater than 255
    def test_fixed_input_upper_limit_rejection(self):
        self.assertRaises(ValueError, mbed_interface.fixed_to_float, 256)

    def test_float_output_is_negative(self):
        for s in (129, 180, 240, 255):
            self.assertLess(mbed_interface.fixed_to_float(s), 0.0)

    def test_float_output_is_positive(self):
        for s in (1, 20, 100, 127):
            self.assertGreater(mbed_interface.fixed_to_float(s), 0.0)

    def test_fixed_input_float_output_pairs(self):
        input_output_pairs = (
                          (0, 0.0),
                          (2, 0.1),
                          (130, -0.1),
                          (16, 1.0),
                          (144, -1.0),
                          (40, 2.5),
                          (168, -2.5),
                          (126, 7.9),
                          (254, -7.9))
        for input_int, output_float in input_output_pairs:
            self.assertAlmostEqual(mbed_interface.fixed_to_float(input_int), output_float, delta=0.0625)


    #===========================#
    #  Test for float_to_fixed  #
    #===========================#
    
    #Makes sure a float is passed in
    def test_float_only_input(self):
        for s in (240, 40, 0):
            self.assertRaises(TypeError, mbed_interface.float_to_fixed, s)

    #Test for upper/lower out of range error
    def test_float_input_upper_limit_rejection(self):
        self.assertRaises(ValueError, mbed_interface.float_to_fixed, 8.0)
    
    def test_float_input_lower_limit_rejection(self):
        self.assertRaises(ValueError, mbed_interface.float_to_fixed, -8.0)
    
    #Test for making sure negative #'s are greater than 128'
    def test_fixed_output_is_negative(self):
        for s in (-7.0, -5.0, -2.5, -0.1):
            self.assertGreater(mbed_interface.float_to_fixed(s), 128)

    #Test for making sure pos. #'s are less than 128'
    def test_fixed_output_is_positive(self):
        for s in (7.0, 5.0, 2.5, 0.1, 0.001):
            self.assertLess(mbed_interface.float_to_fixed(s), 128)
    
    def test_float_input_fixed_output_pairs(self):
        input_output_pairs = (
                      (0.0, 0),
                      (-0.0, 0),
                      (0.1, 2),
                      (-0.1, 130),
                      (1.0, 16),
                      (-1.0, 144),
                      (2.5, 40),
                      (-2.5, 168),
                      (7.9, 126),
                      (-7.9, 254))
        for input_float, output_int in input_output_pairs:
            self.assertEqual(mbed_interface.float_to_fixed(input_float), output_int)



if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestMbedInterface)

