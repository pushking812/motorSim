package dcmotor

import (
	"fmt"
	"math"

	"github.com/pushking812/motorsim/config"
)

// Пакет simulation, структура DCMotor: содержит номинальные параметры мотора,
// используются для расчета текущих значений моделирования, задаются при запуске
// программы из файла конфигурации или командной строки в пакете main
type DCMotor struct {
	voltage          float64 // Номинальное напряжение, V
	power            float64 // Номинальная мощность, W
	current          float64 // Номинальный ток, A
	statorResistance float64 // Статорное сопротивление, Ом
	emConstant       float64 // Электромагнитная постоянная, Вб/А
	statorInductance float64 // Инерция ротора, кг*м^2
	efficiency       float64 // КПД
	maxCurrent       float64 // Максимальный ток. А
	maxSpeed         float64 // Максимальная скорость, оборотов/сек
	torqueConstant   float64 // Момент инерции, кг*м^2
	loadTorque       float64 // Нагрузка, вращающийся объект с моментом сопротивления, Нм
}

// Пакет simulation, структура currentValues: содержит расчетные значения показателей текущего
// шага моделирования, используются для получения средних значений показателей моделирования за ед. времени
type CurrentValues struct {
	pwmVoltage float64 // текущее напряжение pwm
	pwmCurrent float64 // текущий ток pwm
	speed      float64 // текущие обороты
	voltage    float64 // текущее напряжение
	power      float64 // текущая мощность
	current    float64 // текущий ток
	torque     float64 // крутящий момент
	stepIndex  int     // номер шага моделирования
}

// Пакет simulation, структура AverageValues: содержит средние значения показателей моделирования
// за ед. времени, используются для расчета итоговых значений показателей моделирования, а также
// для создания графических или текстовых отчетов
type AverageValues struct {
	PWMVoltage float64 // среднее напряжение pwm
	PWMCurrent float64 // средний ток pwm
	Speed      float64 // средние обороты
	Voltage    float64 // среднее напряжение
	Power      float64 // средняя мощность
	Current    float64 // средний ток
	Torque     float64 // крутящий момент
	StepIndex  int     // номер отчетного периода
}

// Пакет simulation, структура ResultValues: содержит итоговые значения показателей моделирования
// выводятся в качестве отчета после окончания моделирования,
// используются для создания графических или текстовых отчетов
type ResultValues struct {
	pwmVoltage float64 // итоговое напряжение pwm
	pwmCurrent float64 // итоговый ток pwm
	speed      float64 // итоговые обороты
	voltage    float64 // итоговое напряжение
	power      float64 // итоговая мощность
	current    float64 // итоговый ток
	torque     float64 // крутящий момент
	duration   float64 // итоговое время моделирования, ms
}

// Конструктор объекта NewPWMGenerator
func NewDCMotor(cfg *config.Config) *DCMotor {
	return &DCMotor{
		voltage:          cfg.MotConfigs[cfg.MotConfigDefault].Voltage,
		power:            cfg.MotConfigs[cfg.MotConfigDefault].Power,
		current:          cfg.MotConfigs[cfg.MotConfigDefault].Current,
		statorResistance: cfg.MotConfigs[cfg.MotConfigDefault].StatorResistance,
		emConstant:       cfg.MotConfigs[cfg.MotConfigDefault].EMConstant,
		statorInductance: cfg.MotConfigs[cfg.MotConfigDefault].StatorInductance,
		efficiency:       cfg.MotConfigs[cfg.MotConfigDefault].Efficiency,
		maxCurrent:       cfg.MotConfigs[cfg.MotConfigDefault].MaxCurrent,
		maxSpeed:         cfg.MotConfigs[cfg.MotConfigDefault].MaxSpeed,
		torqueConstant:   cfg.MotConfigs[cfg.MotConfigDefault].TorqueConstant,
		loadTorque:       cfg.MotConfigs[cfg.MotConfigDefault].LoadTorque,
	}
}

// Метод  Simulate выполняет моделирование работы мотора.
// Данный метод будет имитировать работу мотора управляемого ШИМ-сигналом
// в течение определенного времени с определенным шагом
func (m *DCMotor) Simulate() {
	s.lock.Lock()
	defer s.lock.Unlock()

	ave := AverageValues{}

	aveNumSteps := s.calcSimTime() // Рассчет длительности и количества шагов моделирования
	aveStepTime := s.simtime.step
	fmt.Printf("aveNumSteps=%d aveStepTime=%f\n", aveNumSteps, aveStepTime)

	const curNumSteps = 1
	curStepTime := s.simtime.step / curNumSteps
	fmt.Printf("curNumSteps=%d curStepTime=%f\n", curNumSteps, curStepTime)

	sumPwmVoltage := 0.0
	sumPwmCurrent := 0.0
	sumSpeed := 0.0
	sumVoltage := 0.0
	sumPower := 0.0
	sumCurrent := 0.0
	sumTorque := 0.0

	for i := 0; int64(i) < aveNumSteps; i++ {
		// Инициализируем переменные для суммирования значений на каждом шаге
		curPwmVoltage := 0.0
		curPwmCurrent := 0.0
		curSpeed := 0.0
		curVoltage := 0.0
		curPower := 0.0
		curCurrent := 0.0
		curTorque := 0.0

		// Запускаем цикл, который будет выполняться в течение указанного количества шагов
		for j := 0; j < curNumSteps; j++ {

			// Создаем структуру для хранения текущих значений на каждом шаге
			cur := CurrentValues{}
			// Вычисляем текущее время
			currentTime := float64(i)*s.simtime.step + float64(j)*curStepTime
			// Вычисляем сигнал ШИМ на текущем шаге
			cur.pwmVoltage = s.pwm.signal(currentTime)
			// Вычисляем напряжение на двигателе на текущем шаге
			cur.voltage = s.motor.calcVoltage(cur.pwmVoltage)
			// Вычисляем ток на двигателе на текущем шаге
			cur.current = s.motor.calcCurrent(cur.pwmVoltage)
			// Вычисляем мощность на двигателе на текущем шаге
			cur.power = s.motor.calcPower(cur.voltage, cur.current)
			// Вычисляем крутящий момент на текущем шаге
			cur.torque = s.motor.calcTorque(curStepTime)
			// Вычисляем скорость вращения двигателя на текущем шаге
			cur.speed = s.motor.calcSpeed(cur.voltage)

			fmt.Printf("Simulate: i=%d currentTime=%f (cur.)pwmVoltage=%0.2f voltage=%0.2f calcCurrent=%0.4f power=%0.2f torque=%0.4f speed=%0.2f\n",
				i, currentTime, cur.pwmVoltage, cur.voltage, cur.current, cur.power, cur.torque, cur.speed)

			// Обновляем текущие значения
			cur.stepIndex = int(j)
			// Добавляем текущие значения в сумму значений на каждом шаге
			curPwmVoltage += cur.pwmVoltage
			curPwmCurrent += cur.current
			curVoltage += cur.voltage
			curPower += cur.power
			curCurrent += cur.current
			curSpeed += cur.speed
			curTorque += cur.torque
			// Добавляем текущие значения в список значений текущего шага
			s.currval = append(s.currval, cur)
		}

		// Вычисляем средние значения на каждом шаге
		ave.PWMVoltage = curPwmVoltage / float64(curNumSteps)
		ave.PWMCurrent = curPwmCurrent / float64(curNumSteps)
		ave.Voltage = curVoltage / float64(curNumSteps)
		ave.Power = curPower / float64(curNumSteps)
		ave.Current = curCurrent / float64(curNumSteps)
		ave.Speed = curSpeed / float64(curNumSteps)
		ave.Torque = curTorque / float64(curNumSteps)

		sumPwmVoltage += ave.PWMVoltage
		sumPwmCurrent += ave.PWMCurrent
		sumVoltage += ave.Voltage
		sumPower += ave.Power
		sumCurrent += ave.Current
		sumSpeed += ave.Speed
		sumTorque += ave.Torque

		s.AverVal = append(s.AverVal, ave)
	}

	// Записываем средние значения в структуру Simulation
	s.RestVal = &ResultValues{
		pwmVoltage: sumPwmVoltage / float64(aveNumSteps),
		pwmCurrent: sumPwmCurrent / float64(aveNumSteps),
		voltage:    sumVoltage / float64(aveNumSteps),
		power:      sumPower / float64(aveNumSteps),
		current:    sumCurrent / float64(aveNumSteps),
		speed:      sumSpeed / float64(aveNumSteps),
		torque:     sumTorque / float64(aveNumSteps),
		duration:   float64(aveNumSteps*curNumSteps) * curStepTime,
	}
}

// Вычисление временных параметров моделирования
func (m *DCMotor) calcSimTime() int64 {
	period := 1 / float64(s.pwm.frequency)
	dutyLowState := 1 - s.pwm.duty/100
	s.simtime.step = (period * dutyLowState) / 2 // длительность шага моделирования

	// количество шагов моделирования
	return int64(s.simtime.duration / s.simtime.step)
}

// Метод вычисляет текущеее напряжение на двигателе
func (m *DCMotor) calcVoltage(pwmV float64) float64 {
	return pwmV
}

// Метод вычисляет текущий ток потребляемый двигателем
func (m *DCMotor) calcCurrent(v float64) float64 {
	// вычисляем с переводом оборотов/сек в радианы/сек
	return (v - m.emConstant*m.maxSpeed/(2*math.Pi)) / m.statorResistance
}

// Метод вычисляет текущую потребляемую мощность двигателя
func (m *DCMotor) calcPower(v float64, i float64) float64 {
	return v * i * m.efficiency
}

// Метод calcSpeed рассчитывает скорость вращения двигателя на основе переданной мощности и крутящего момента.
// Вычисления производятся в соответствии с параметрами моделирования, заданными в структуре Simulation.
// Результат вычислений - текущая скорость вращения двигателя в об/сек.
func (m *DCMotor) calcSpeed(voltage float64) float64 {
	const c1 = 0.0037
	const c2 = 0.0
	const c3 = 0.047
	const c4 = 0.0000079
	inertia := 0.1

	i := voltage / s.nominal.statorResistance
	w := s.nominal.maxSpeed * 2 * math.Pi
	for t := 0.0; t < s.simtime.duration; t += s.simtime.step {
		P := c1*w + c2*w*w
		T := c3*i - c4*w*i
		dw := (P - T*w) / inertia
		w += dw * s.simtime.step
	}
	return w
}

func (m *DCMotor) calcTorque(steptime float64) float64 {
	const c1 = 0.03
	const c2 = 0.03
	const c3 = 0.015
	const c4 = 0.00005

	return c1*steptime + c2*math.Pow(steptime, 2) + c3*m.current + c4*m.current*steptime
}
