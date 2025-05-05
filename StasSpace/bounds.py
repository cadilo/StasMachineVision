import numpy as np
from ikpy.chain import Chain
from itertools import product

# Загрузка цепи манипулятора
chain = Chain.from_urdf_file("description.urdf", active_links_mask=[0, 1, 1, 1, 1, 1, 1])

# Определение количества шагов и ограничений для каждого сустава
steps = 10
joint_limits = [
    (-np.inf, np.inf),    # Joint 0 (fixed)
    (0.0, 2 * np.pi),     # Joint 1 (revolute)
    (0.6108, 1.13446),    # Joint 2 (revolute)
    (0.95993, 1.5708),    # Joint 3 (revolute)
    (0.0, 4.7123),        # Joint 4 (revolute)
    (0.0, np.pi),         # Joint 5 (revolute)
    (0.0, 4.71239)        # Joint 6 (revolute)
]

# Генерация значений суставов
joint_values = []
for joint_limit in joint_limits:
    if joint_limit[0] == -np.inf and joint_limit[1] == np.inf:
        joint_values.append([0])  # Фиксированный сустав
    else:
        joint_values.append(np.linspace(joint_limit[0], joint_limit[1], steps))

# Получение всех комбинаций суставов
joint_combinations = product(*joint_values)

# Инициализация диапазонов
min_position = np.full(3, np.inf)
max_position = np.full(3, -np.inf)

# Вычисление диапазонов координат конечного эффектора
for combination in joint_combinations:
    position = chain.forward_kinematics(combination)[:3, 3]  # Получение позиции XYZ конечного эффектора
    
    # Обновление минимальных и максимальных значений
    min_position = np.minimum(min_position, position)
    max_position = np.maximum(max_position, position)

# Печать диапазонов значений
print("Range of End-Effector Positions:")
print(f"X: {min_position[0]} to {max_position[0]}")
print(f"Y: {min_position[1]} to {max_position[1]}")
print(f"Z: {min_position[2]} to {max_position[2]}")