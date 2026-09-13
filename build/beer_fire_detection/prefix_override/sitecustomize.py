import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jordan/G25_RS1/RS1-Gr25/install/beer_fire_detection'
