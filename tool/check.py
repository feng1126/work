# -*- coding: utf-8 -*-

import os
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='rename file name.')
    
    parser.add_argument('folder',  metavar='folder',nargs='?', help='Data folder')
    parsed = parser.parse_args()
    print(parsed.folder)
    TEST_IMAGES = os.listdir(parsed.folder)
    # TEST_IMAGES.sort()
    print("number of image " + str(len(TEST_IMAGES)))