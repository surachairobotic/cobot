import sympy as sp
import numpy as np
import math
import time
import pickle
import lib_sympy as lib_sp
import lib_equation as lib_eq

import create_eq
import set_const
import create_txt

joint_num = 6
create_eq.run(joint_num)
set_const.run(joint_num,True)
create_txt.run(joint_num,True)
