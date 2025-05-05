import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot
from ikpy.chain import Chain  # Замените 'your_module' на ваш модуль, где определен класс Chain
from ikpy.utils.geometry import from_transformation_matrix, to_transformation_matrix
from chain import checking_for_availability

class RobotAnimation:
    def __init__(self):
        chain = Chain.from_urdf_file("description.urdf", active_links_mask=[0, 1, 1, 1, 1, 1, 1])
        ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
        chain.plot(target_joint_angels, ax, target=pk, show=True)
        pass

# Использование класса
if __name__ == "__main__":

