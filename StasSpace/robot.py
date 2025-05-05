import numpy as np 
from port import Port
from chain import inverse_kinematic, forward_kinematic
import argparse
import json
import time

# Данные для калибровки
# для J0 - 1 градус = 7 шагов
# для J1 - 1 градус = 300 шагов
# для J2 - 1 градус = 285 шагам

#usage_port = '/dev/pts/8'
usage_port = '/dev/ttyUSB0'
#usage_port = None

class RobotCalibration():
    k = np.array([
        7, 
        300,
        285,
        -1,
        1,
        1,
    ])

    def steps_to_degrees(self, units=(0, 0, 0, 0, 0, 0)):
        return int([ round(x,4) for x in np.array(units) / self.k ])

    def degrees_to_steps(self, degrees=(0, 0, 0, 0, 0, 0)):
        return [ round(x,4) for x in np.array(degrees) * self.k ]

class Robot():
    def __init__(self, port=usage_port, timeout=10000):
        self.port = Port(port)
        self.calibration = RobotCalibration()
        self.timeout = timeout

    def set_coord_pos(self, pos):
        ik1 = inverse_kinematic(pos[0], pos[1])
        j0, j1, j2, j3, j4, j5 = [ round(np.degrees(x),2) for x in ik1 ][1:7]
        robot.set_joint_pos((j0, j1, j2, j3, j4, j5))

    def set_joint_pos(self, joints=(0, 0, 0, 0, 0, 0)):
        
        joints = self.calibration.degrees_to_steps(joints)
        j1, j2, j3, j4, j5, j6 = [int(joints[i]) for i in range(6)]
        self.port.G00(j1, j2, j3, j4, j5, 90)
        time.sleep(10)

    def set_zero_pos(self):
        self.port.G00(0, 0, 0, 0, 0, 0)
        print(self.port.G01())

    def start_programm(self, filename):
        print("Вызван метод start_programm")

        try:
            # Читаем позиции из JSON файла
            with open(filename, 'r') as json_file:
                positions = json.load(json_file)

            for position in positions:
                # Формируем команду G00
                self.port.G00(position[0][1], position[1][1], position[2][1], position[3][1], position[4][1], position[5][1])
                time.sleep(2)
                # if self.port.G01() == "position complete":
                #     continue
                # else:
                #     print("Позиция не достигнута")

        finally:
            self.port.G00(0, 0, 0, 0, 0, 0)
            #print('Последовательный порт закрыт.')

    def write_programm(self, filename):
        print("Вызван метод write_programm")
        positions = []  # Объявляем positions как список
        try:
            while True:
                command = input("Введите write для записи положения (или 'exit' для завершения): ")
                if command.lower() == 'exit':
                    break
                
                # Отправляем команду
                response = self.port.G01()
                # Получаем ответ от устройства
                print(f'Ответ получен: {response}')
                
                response_data = []
                for pair in response.split():
                    if ':' in pair:  # Проверяем наличие ':' в строке
                        key, value = pair.split(':')
                        key = key.strip()  # Убираем лишние пробелы
                        value = value.strip()  # Убираем лишние пробелы
                        if value:  # Проверяем, что value не пустое
                            if value.isdigit():  # Проверка, что value является числом
                                response_data.append((key, int(value)))  
                            else:
                                print(f"Неправильное значение: {value}")  # Предупреждение, если значение не целое число
                        else:
                            print(f"Пустое значение для ключа: {key}")  # Уведомление о пустом значении
                    else:
                        print(f"Неправильный формат ответа: {pair}")

                positions.append(response_data)  # Добавляем response_data в positions

        finally:
            # Сохраняем ответы в JSON файл
            with open(f'{filename}', 'w') as json_file:
                json.dump(positions, json_file, indent=4)  # Сохраняем в формате JSON
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    ARGS = [
        ('-X',  '--X',  float,   None, 'X position M.'),
        ('-Y',  '--Y',  float, None, 'Y position M.'),
        ('-Z',  '--Z',  float, None, 'Z position M.'),
        ('-rX', '--rX', float,   None, 'X tool rotation, degrees.'),
        ('-rY', '--rY', float,   None, 'Y tool rotation, degrees.'),
        ('-rZ', '--rZ', float,   None, 'Z tool rotation, degrees.'),
        ('-rot','--rot',  str, 'Z', 'Tool rotation axis.'),
        ('-j0', '--j0', int,   0, 'Joint #0 position, degrees.'),
        ('-j1', '--j1', int,   0, 'Joint #1 position, degrees.'),
        ('-j2', '--j2', int,   0, 'Joint #2 position, degrees.'),
        ('-j3', '--j3', int,   0, 'Joint #3 position, degrees.'),
        ('-j4', '--j4', int,   0, 'Joint #4 position, degrees.'),
        ('-j5', '--j5', int,   0, 'Joint #5 position, degrees.'),
        ('-s', '--speed', int,  70, 'Speed, percent.'),
        ('-sp', '--start_programm', str, None, 'Start programm filename'),
        ('-wp', '--write_programm', str, None, 'Write programm filename'),
        ('-g01', '--G01', str, None, 'Send command G01'),
        ('-xt1', '--xt1', int, None, 'Passing by position'),
        ('-yt1', '--yt1', int, None, 'Passing by position')
    ]
    try:
        robot = Robot()
    except Exception:
        print('working in emulated mode')
        robot = Robot(port=None)
    for a_short, a_long, a_type, a_value, a_help in ARGS:
        parser.add_argument(a_short, a_long, type=a_type, default=a_value, help=a_help)
    
    args = parser.parse_args()
    print(args)
    j0 = args.j0
    j1 = args.j1
    j2 = args.j2
    j3 = args.j3
    j4 = args.j4
    j5 = args.j5
    X = args.X
    Y = args.Y
    Z = args.Z
    rX = args.rX
    rY = args.rY
    rZ = args.rZ
    rot = args.rot
    speed = args.speed
    start_filename = args.start_programm
    write_filename = args.write_programm
    G01 = args.G01
    xt1 = args.xt1
    yt1 = args.yt1

    vec = [X, Y, Z]
    rot = [rX, rY, rZ]

    pos_1 = [[-0.67, 0.0, 0.16], [-180, -5, 180]]   #[[coordinate], [orientation]]

    pos_2 = [[-0.67, 0.0, -0.1], [-180, -5, 180]]



    if start_filename is not None:
        robot.start_programm(start_filename)
    elif write_filename is not None:
        robot.write_programm(write_filename)
    elif G01 is not None:
        robot.port.G01()
    elif xt1 and yt1 is not None:
        #Change PORT for Start
        #robot.set_coord_pos(pos_1)
        #robot.set_coord_pos(pos_2)
        #robot.port.G05(1)
        #robot.set_coord_pos(pos_1)
        #robot.port.G05(0)
        #robot.set_zero_pos()
        
        #from forvova import coords

        #path = "/home/cnc/Stanislav/robo/robik/best.pt"
        #camera_vision = coords(path)
        
        #x_botle_cam, y_botle_cam = camera_vision.coordinates()
        x_botle_cam, y_botle_cam = xt1, yt1
        print(f"В притирочку: X={x_botle_cam:.2f}cm, Y={y_botle_cam:.2f}cm")
        pos_work_table = [ 0.24, -0.48, -0.09] # Change value
        x_botle = round(0.24 + (y_botle_cam/100), 2)
        y_botle = round(-0.40 - (x_botle_cam/100), 2)
        coor_botle = [-x_botle, y_botle, 0.16]
        #coor_botle = [1, 1, 1]
        print("Coord_botle: ", coor_botle)
        #pos=[[-0.35, -0.55, 0.14], [-180, -5, 180]] #Change orientation
        robot.port.G00(0, 0, 0, 0, 0, 90)
        pos=[coor_botle, [-180, -5, -180]] #Change orientation
        camera_vision=0
        
        robot.set_coord_pos(pos)
        robot.port.G05(1)
        coor_botle = [-x_botle, y_botle, 0.20]
        pos=[coor_botle, [-180, -5, -180]] #Change orientation
        robot.set_coord_pos(pos)
        
        robot.set_coord_pos(pos_1)
        robot.port.G05(0)

    else:
        if sum([j0, j1, j2, j3, j4, j5]):
            print('joints')
            
            fk = forward_kinematic(np.radians(j0), 
                                   np.radians(j1),
                                   np.radians(j2), 
                                   np.radians(j3), 
                                   np.radians(j4), 
                                   np.radians(j5))
            robot.set_joint_pos((j0, j1, j2, j3, j4, j5))
            time.sleep(10)
            robot.set_joint_pos((0, 0, 0, 0, 0, 0))
            
        else:
            print('coordinates')
            print(vec, rot)
            ik = inverse_kinematic(vec, rot)
            j0, j1, j2, j3, j4, j5 = [ round(np.degrees(x),2) for x in ik ][1:7]
            print(j0, j1, j2, j3, j4, j5)
            robot.set_joint_pos((j0, j1, j2, j3, j4, j5))
            robot.set_zero_pos()

