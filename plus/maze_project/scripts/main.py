#!/usr/bin/env python

import rospy
import escape_maze

rospy.init_node('maze_module')
es = escape_maze.Begin()

if __name__ == "__main__":
    es.get_target()
