import matplotlib.pyplot

from ikpy.chain import Chain
from ikpy.inverse_kinematics import inverse_kinematic_optimization
from ikpy.utils.geometry import from_transformation_matrix, to_transformation_matrix
import numpy as np
chain = Chain.from_urdf_file("description.urdf", active_links_mask=[0, 1, 1, 1, 1, 1, 1, 0])

def eul2rot(alpha, beta, gamma):
    # Преобразуем углы из градусов в радианы
    alpha = np.radians(alpha)  # Roll
    beta = np.radians(beta)    # Pitch
    gamma = np.radians(gamma)  # Yaw
    
    # Вычисляем компоненты матрицы поворота
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(alpha), -np.sin(alpha)],
                   [0, np.sin(alpha), np.cos(alpha)]])

    Ry = np.array([[np.cos(beta), 0, np.sin(beta)],
                   [0, 1, 0],
                   [-np.sin(beta), 0, np.cos(beta)]])

    Rz = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                   [np.sin(gamma), np.cos(gamma), 0],
                   [0, 0, 1]])
    
    # Итоговая матрица поворота (R = Rz * Ry * Rx)
    R = Rz @ (Ry @ Rx)
    return R

def rot2eul(R):
    beta = np.arcsin(-R[2, 0])
    alpha = np.arctan2(R[2, 1] / np.cos(beta), R[2, 2] / np.cos(beta))
    gamma = np.arctan2(R[1, 0] / np.cos(beta), R[0, 0] / np.cos(beta))
    return [np.degrees(x) for x in [alpha, beta, gamma]]

def checking_for_availability(j1, j2, j3, j4, j5, j6):
    print(f'Check joints: {j1, j2, j3, j4, j5, j6}')
    joint_limits = [
    (-1.57, 1.57),     # Joint 1 (revolute)
    (0.0, 0.6109),    # Joint 2 (revolute)
    (0.0, 0.6109),    # Joint 3 (revolute)
    (-3.14, 0.0),        # Joint 4 (revolute)
    (-1.57, 1.57),         # Joint 5 (revolute)
    (0.0, 3.1416),        # Joint 6 (revolute)
    ]

    joint_values = [j1, j2, j3, j4, j5, j6]

    for i, value in enumerate(joint_values):
        lower_limit, upper_limit = joint_limits[i]
        if not (lower_limit <= value <= upper_limit):  # Проверка, входит ли значение в пределы
            return False
    return True  # Все значения в пределах

def forward_kinematic(j1, j2, j3, j4, j5, j6):
    print(f"Debug: {j1, j2, j3, j4, j5, j6}")
    flag = checking_for_availability(j1, j2, j3, j4, j5, j6)
    print(f'flag = {flag}')
    if flag:
        print("Значения входят в допустимый диапазон")
    else:
        print("Значения выходят за допустимый диапазон")

    target_joint_angels = [0, j1, j2, j3, j4, j5, j6, 0]
    pk = chain.forward_kinematics(target_joint_angels)
    print(f"{pk}") 
    Vector, Rotate = from_transformation_matrix(pk)

    #print(f"Vector non normilize: {Vector} \n\r")
    coor_normilize = [f"{v:.2f}" for v in Vector]
    print(f"Vector normilize: {coor_normilize} \n\r")


    #print(f"Rotate non normilize: {Rotate} \n\r")
    rotate_normilize = rot2eul(Rotate)
    print(f"Rotate normilize: {rotate_normilize} \n\r")


    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
    chain.plot(target_joint_angels, ax, target=pk, show=True)
    matplotlib.pyplot.show()


def out_red(text):
    print("\033[34m{}".format(text))


def inverse_kinematic(vector, rotates):
    initial_joint_angels = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00] #Домашняя позиция 
    
    
    rotate = eul2rot(rotates[0], rotates[1], rotates[2])
    target_frame = to_transformation_matrix(vector, rotate)
    print(target_frame)

    invers_opt = inverse_kinematic_optimization(chain, target_frame, initial_joint_angels, orientation_mode="all")
    print(f"Invers non normilized: {invers_opt}")

    joint_normilized = [f"{v:.2f}" for v in invers_opt]

    flag = checking_for_availability(float(joint_normilized[1]), 
                                     float(joint_normilized[2]), 
                                     float(joint_normilized[3]),
                                     float(joint_normilized[4]),
                                     float(joint_normilized[5]),
                                     float(joint_normilized[6]))
    
    print(f'flag = {flag}')
    if flag:
        print("Значения входят в допустимый диапазон")
    else:
        print("Значения выходят за допустимый диапазон")
    print(f"invers normilized: {joint_normilized}")

    print(f'float result debug {[float(joint_normilized[i]) for i in range(4)]}')

    result = [initial_joint_angels[i] - float(joint_normilized[i]) for i in range(4)]
    result.extend(float(joint_normilized[i]) for i in range(-4, -1))

    print(f'Result delta: {result}\n')

    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
    chain.plot(invers_opt, ax)
    matplotlib.pyplot.show()
    return result


if __name__ == '__main__':
    target_vector = [-0.67, 0.0, 0.16]
    target_rotate = [-180, -5, 180]
    inverse_kinematic(target_vector, target_rotate)

    target_joint = [0.0, 0.0, 0.0, 0.00, 1.56, 0.00]
    #forward_kinematic(target_joint[0], target_joint[1], target_joint[2], target_joint[3], target_joint[4], target_joint[5])