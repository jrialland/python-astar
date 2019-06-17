import unittest

import tests.basic
import tests.london
import tests.tan_network

def suite():
    suite = unittest.TestSuite()
    suite.addTest(tests.basic.BasicTests())
    suite.addTest(tests.london.LondonTests())
    suite.addTest(tests.tan_network.TanFinderTests())
    return suite
