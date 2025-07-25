import matplotlib.pyplot as plt

#Класс симуляции
class Simulation():
    def __init__(self,drone,controller):
        self.drone = drone # Математическая модель дрона,параметры которого будут использоваться в симуляции
        self.controller = controller # Контроллер для управления двигателями дрона
    def start_simulation(self,start_speed,target_speed,dt,simulation_time = 10):
        current_time = 0
        current_speed = start_speed
        speed_data = [start_speed]
        thrust_data = [0.5]
        acc_data =[0]
        while current_time < simulation_time:
            thrust = self.controller.calc_thrust(current_speed,target_speed) # Расчет тяги двигателя
            acc = self.drone.get_acc(current_speed,thrust) # Расчет ускорения
            current_speed+=acc*dt # Интегрируем ускорение и получаем изменение скорости
            # Добавляем текущие параметры полета в списки для дальнейшего анализа
            acc_data.append(acc)
            speed_data.append(current_speed)
            thrust_data.append(self.drone.real_thrust)
            current_time+=dt # Прибавляем к текущему времени шаг симуляции
        return acc_data,speed_data,thrust_data,controller.Pdata,controller.Idata,controller.Ddata
    
class Drone():
    def __init__(self,max_engine_power,mass,start_real_thrust=0.5,engine_inertion = 0.1):
        self.max_engine_power = max_engine_power # Максимальная тяга двигателей в Ньютонах
        self.mass = mass # Масса дрона
        self.real_thrust = start_real_thrust # Фактическая тяга двигателей дрона в диапазоне от -0.2 до 1(начальное значение по умолчанию равна 0)
        self.engine_inertion = engine_inertion # Коэффициент инерции двигателей. Характеризует скорость изменения тяги
    def get_acc(self,speed,command_thrust):# Расчет ускорения дрона
        self.real_thrust += (command_thrust-self.real_thrust)*self.engine_inertion# Расчет изменения фактической тяги с учетом инерции двигателей
        engine_power = self.max_engine_power * self.real_thrust# Расчет мощности двигателей путем перевода тяги в Ньютоны 
        force = engine_power - self.mass*9.8# Расчет равнодействующей сил действующих на дрон как разницы между силой тяги и силы тяжести
        return force/self.mass# Расчитываем и возвращаем ускорение
    
class Contoller():
    def __init__(self,P,I,D,dt):
        self.P = P
        self.I = I
        self.D = D
        self.dt = dt
        self.past_error = None
        self.stored_error = 0
        self.Pdata = []
        self.Ddata = []
        self.Idata = []
    def calc_thrust(self,current_speed,target_speed, max_thrust = 1, min_thrust = 0.1):
        error = target_speed - current_speed # Расчет ошибки
        if self.past_error == None:
            self.past_error = error
        # Расчет каждой компоненты PID регулятора
        P_comp = error*self.P
        I_comp = self.stored_error*self.I*self.dt
        D_comp = (error-self.past_error)/self.dt * self.D
        # Расчет требуемой тяги
        thrust =  P_comp + D_comp + I_comp
        # Ограничение уровня тяги. Тяга не может быть больше 1 или меньше 0.1(холостые обороты) 
        if thrust > max_thrust:
            thrust = max_thrust
        if thrust < min_thrust:
            thrust = min_thrust
        self.past_error = error
        self.stored_error += error*dt# Расчитываем учитываем накопление ошибки
        # Сохраняем компоненты в список для дальнейшего анализа
        self.Pdata.append(P_comp)
        self.Idata.append(I_comp)
        self.Ddata.append(D_comp)
        return thrust


# Параметры симуляции
max_engine_power = 20# Максимальныая тяга двигателей в Ньютонах
mass = 1# Масса дрона
dt = 0.1# шаг симуляции

# Один контроллер
drone = Drone(max_engine_power,mass)# Создаем модель дрона с заданными параметрами
controller = Contoller(0.5,0.3,0.5,dt)# Создаем контроллер с заданными коэффициентами

# Запускаем симуляцию
sim = Simulation(drone,controller)
# Собираем данные симуляции
acc_data,speed_data,thrust_data,Pdata,Idata,Ddata = sim.start_simulation(0,10,dt)



fig, ((ax1,ax2,ax4)) = plt.subplots(1, 3, figsize=(20, 8))
# График скорости
ax1.plot(speed_data,label = "spd")
ax1.set_title('Cкорость')
ax1.legend()  
ax1.grid(True)

# График тяги
ax2.plot(thrust_data,color = "red", label = "thrust")
ax2.set_title('Тяга')
ax2.legend()  
ax2.grid(True)
'''
# График ускорения
ax3.plot(acc_data,color = "green", label="acc")
ax3.set_title('Ускорение')
ax3.legend()  
ax3.grid(True)'''

# Графики компонент ПИД регулятора
ax4.plot(Pdata,color = "red", label="P")
ax4.plot(Idata,color = "green", label="I")
ax4.plot(Ddata,color = "blue", label="D")
ax4.legend()  
ax4.grid(True)

ax4.set_title('PID')

plt.show()




 # Сравнение
PID_list = [[0.5,0,0],[0.5,0,0.5],[0.5,0.3,0.5]]
fig, ((ax1,ax2),(ax3,ax4)) = plt.subplots(2, 2, figsize=(10, 8))

for PID in PID_list:
    
    drone = Drone(max_engine_power,mass)
    controller = Contoller(PID[0],PID[1],PID[2],dt)
    sim = Simulation(drone,controller)
    acc_data,speed_data,thrust_data,Pdata,Idata,Ddata = sim.start_simulation(0,10,dt)

    label = f"PID (P={PID[0]}, I={PID[1]}, D={PID[2]})"
    ax1.plot(speed_data,label = label)
    ax1.set_title('скорость')

    ax2.plot(thrust_data, label = label)
    ax2.set_title('Тяга')

    ax3.plot(acc_data, label=label)
    ax3.set_title('Ускорение')

# Добавляем заголовки и легенды
ax1.set_title('Скорость')
ax1.legend()  
ax1.grid(True)  

ax2.set_title('Тяга')
ax2.legend()  
ax2.grid(True)

ax3.set_title('Ускорение')
ax3.legend()  
ax3.grid(True)

plt.tight_layout()  # Автоматически регулируем отступы
plt.show()


