import sys
import os

sys.path.append(os.path.abspath("../"))

from dbw_node import DBWNode

def main():
    print("twist_controller_test")
    node = DBWNode()
    node.loop()

if __name__ == '__main__':
    main()
