import dvrk

p = dvrk.psm("PSM1")

# now you can home from Python
p.enable()
p.home()


# move multiple joints
import numpy as np
# move in cartesian space
import PyKDL
# start position
goal = p.setpoint_cp()
# move 5cm in z direction
goal.p[2] += 0.05
p.move_cp(goal).wait()

import math
# start position
goal = p.setpoint_cp()
# rotate tool tip frame by 25 degrees
goal.M.DoRotX(math.pi * 0.25)
p.move_cp(goal).wait()