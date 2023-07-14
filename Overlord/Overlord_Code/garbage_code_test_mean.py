import random
from statistics import mean as mu
from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt
import csv
# import seaborn as sns; sns.set()
# import pandas as pd
# from random import randrange

import math
# from numpy.random import normal
from numpy.random import seed
from scipy.stats import norm
# from tabulate import tabulate

from scipy.integrate import simps
from numpy import trapz
from matplotlib.animation import FuncAnimation
from random import randint
from csv import reader

data = [5, 5, 7, 23, 8, 9, 43, 12, 5, 8]
print(np.array(data).mean())
