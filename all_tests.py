#!/usr/bin/env python3


if __name__ == "__main__":
	import sys
	import unittest
	import tests
	runner = unittest.TextTestRunner()
	sys.exit(runner.run(tests.suite()))
