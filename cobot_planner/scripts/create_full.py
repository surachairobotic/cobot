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
create_eq.exec(joint_num)
set_const.exec(joint_num,True)
create_txt.exec(joint_num,True)
