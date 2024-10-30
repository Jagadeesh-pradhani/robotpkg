import os

with open('old.urdf', 'r') as infp:
        robot_desc = infp.read()
with open('converted.urdf', 'w') as outfp:
        outfp.write(robot_desc)
