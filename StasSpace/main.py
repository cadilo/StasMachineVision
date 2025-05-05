from PyQt5 import QtWidgets, QtCore
from design import Ui_MainWindow
import sys
import port
import time


class mywindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(mywindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.realport = None
        self.ui.ComboBox_Port.addItems(port.serial_ports())

        self.ui.Button_Connect.clicked.connect(self.Connect)
        self.ui.pushButton.clicked.connect(self.Read)
        self.ui.pushButton_2.clicked.connect(self.Write)


    def Connect(self):
        try:
            self.realport = port.serial.Serial(self.ui.ComboBox_Port.currentText(), 9600)
            if self.realport.is_open:
                self.ui.Button_Connect.setStyleSheet("background-color: green")
                self.ui.Button_Connect.setText('Подключено')

            else:
                self.ui.Button_Connect.setStyleSheet("background-color: red")
                self.ui.Button_Connect.setText('Не подключено')
        except Exception as e:
            print(e)
        except port.serial.SerialException as e:
            print(f"Ошибка соединения: {e}")


    def Read(self):
        if self.realport is not None:
            message = f"1\n"  # Создание сообщения
            self.realport.write(message.encode('utf-8'))  # Кодирование строки в байты и отправка
            data = self.realport.readline().decode('utf-8').rstrip()  # Чтение строки
            print(data)  # Вывод данных на экран
            time.sleep(1)
            # if self.realport.in_waiting > 0:  # Проверка, есть ли доступные данные
            #     data = self.realport.readline().decode('utf-8').rstrip()  # Чтение строки
            #     print(data)  # Вывод данных на экран
            # else:
            #     print("Доступных данных нет")
    def Write(self):
        message = f"Сообщение \n"  # Создание сообщения
        self.realport.write(message.encode('utf-8'))  # Кодирование строки в байты и отправка
        print(f"Отправлено: {message.strip()}")  # Вывод отправленного сообщения
        time.sleep(1)  # Задержка между отправками
 
 
app = QtWidgets.QApplication([])
application = mywindow()
application.show()

sys.exit(app.exec())