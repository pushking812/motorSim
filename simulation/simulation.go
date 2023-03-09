// Пакет реализует симуляцию работы мотора управляемого PWM-генератором
package simulation

import (
	"errors"
	"sync"
	"time"

	"github.com/pushking812/motorSim/config"
)

// Структура Simulation, пакета simulation: содержит параметры симуляции
type Simulation struct {
	nominal *dcMotor      // параметры симуляции, мотор
	env     *environment  // параметры симуляции, окружающая среда
	pwm     *pwmGenerator // параметры симуляции, PWM-генератор
	simtime *simTime      // параметры симуляции, время и шаг симуляции

	currval []currentValues // текущие значения параметров при симуляции, за шаг симуляции
	AverVal []AverageValues // средние значения параметров при симуляции, за период времени
	RestVal *ResultValues   // итоговые значения параметров при симуляции, итоговые за симуляцию

	lock sync.RWMutex
}

// Конструктор объекта Simulation. Определяет параметры симуляции:
// характеристики мотора, окружающей среды, PWM-генератора
func NewSimulation(cfg *config.Config) (*Simulation, error) {
	if cfg == nil {
		return nil, errors.New("NewSimulation: Can't create object Simulation")
	}

	// Возвращаем указатель на созданный экземпляр симуляции
	return &Simulation{
		nominal: NewDCMotor(cfg),
		env:     NewEnvironment(cfg),
		simtime: NewSimTime(cfg),
		currval: make([]currentValues, 0),
		AverVal: make([]AverageValues, 0),
		RestVal: &ResultValues{},
		pwm:     NewPWMGenerator(cfg),
	}, nil
}

// Пакет simulation, структура dcMotor: содержит номинальные параметры мотора,
// используются для расчета текущих значений симуляции, задаются при запуске
// программы из файла конфигурации или командной строки в пакете main
type dcMotor struct {
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

// Конструктор объекта NewPWMGenerator
func NewDCMotor(cfg *config.Config) *dcMotor {
	return &dcMotor{
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

// Пакет simulation, структура environment: содержит параметры окружающей среды используюемые
// для расчета текущих значений симуляции, задаются при запуске программы из файла
// конфигурации или командной строки в пакете main
type environment struct {
	temperature int // Температура окружающей среды, °C
	humidity    int // Влажность, %
	pressure    int // Атмосферное давление
}

// Пакет simulation: инициализация параметров окружающей среды
func NewEnvironment(cfg *config.Config) *environment {
	return &environment{
		temperature: cfg.EnvConfigs[cfg.EnvConfigDefault].Temperature,
		humidity:    cfg.EnvConfigs[cfg.EnvConfigDefault].Humidity,
		pressure:    cfg.EnvConfigs[cfg.EnvConfigDefault].Pressure,
	}
}

// время и шаг симуляции
type simTime struct {
	duration int64 // длительность симуляции
	step     int64 // шаг симуляции
}

// время и шаг симуляции
func NewSimTime(cfg *config.Config) *simTime {
	return &simTime{
		duration: cfg.SimConfigs[cfg.SimConfigDefault].Duration,
		step:     cfg.SimConfigs[cfg.SimConfigDefault].Step,
	}
}

// Структура пакета simulation: параметры ШИМ-генератора.
type pwmGenerator struct {
	minVoltage float64 // Напряжение пассивной фазы
	maxVoltage float64 // Напряжение активной фазы ШИМ-генератора
	duty       float64 // Скважность ШИМ-генератора
	frequency  int64   // Частота ШИМ-генератора
}

// Конструктор пакета simulation: инициализация параметров ШИМ-генератора
func NewPWMGenerator(cfg *config.Config) *pwmGenerator {
	return &pwmGenerator{
		minVoltage: cfg.PWMConfigs[cfg.PWMConfigDefault].MinVoltage,
		maxVoltage: cfg.PWMConfigs[cfg.PWMConfigDefault].MaxVoltage,
		duty:       cfg.PWMConfigs[cfg.PWMConfigDefault].Duty,
		frequency:  cfg.PWMConfigs[cfg.PWMConfigDefault].Frequency,
	}
}

// Пакет simulation, структура currentValues: содержит расчетные значения показателей текущего
// шага симуляции, используются для получения средних значений показателей симуляции за ед. времени
type currentValues struct {
	pwmVoltage float64 // текущее напряжение pwm
	pwmCurrent float64 // текущий ток pwm
	speed      float64 // текущие обороты
	voltage    float64 // текущее напряжение
	power      float64 // текущая мощность
	current    float64 // текущий ток
	stepIndex  int     // номер шага симуляции
}

// Пакет simulation, структура AverageValues: содержит средние значения показателей симуляции
// за ед. времени, используются для расчета итоговых значений показателей симуляции, а также
// для создания графических или текстовых отчетов
type AverageValues struct {
	PWMVoltage float64 // среднее напряжение pwm
	PWMCurrent float64 // средний ток pwm
	Speed      float64 // средние обороты
	Voltage    float64 // среднее напряжение
	Power      float64 // средняя мощность
	Current    float64 // средний ток
	StepIndex  int     // номер отчетного периода
}

func (s *Simulation) GetAveValues() []AverageValues {
	return s.AverVal
}

// Пакет simulation, структура ResultValues: содержит итоговые значения показателей симуляции
// выводятся в качестве отчета после окончания симуляции,
// используются для создания графических или текстовых отчетов
type ResultValues struct {
	pwmVoltage float64 // итоговое напряжение pwm
	pwmCurrent float64 // итоговый ток pwm
	speed      float64 // итоговые обороты
	voltage    float64 // итоговое напряжение
	power      float64 // итоговая мощность
	current    float64 // итоговый ток
	duration   int     // итоговое время симуляции, ms
}

// Пакет simulation: метод симуляции работы PWM-генератора.
// Метод возвращает напряжение PWM-генератора в момент времени elapsed
// прошедший с начала симуляции, для вычисления напряжения использует
// напряжение, скважность и частоту генератора. Форма сигнала - меандр.
// напряжение меняется от pwm.minVoltage до pwm.maxVoltage,
// скважность pwm.pwmDuty - значение от 0 до 100,
// частота pwm.pwmFrequency - единица измерения Hz
// elapsed - единица измерения миллисекунды
func (pwm *pwmGenerator) signal(elapsed int64) float64 {
	// Вычисляем период меандра в миллисекундах, где 1e3 - количество миллисекунд в одной секунде
	period := 1e3 / pwm.frequency

	// Вычисляем время, прошедшее с начала текущего периода меандра
	t := elapsed % period

	// Вычисляем скважность меандра в долях единицы (от 0 до 1)
	duty := float64(pwm.duty) / 100

	// Определяем, находится ли текущий момент времени внутри активной фазы меандра
	inActivePhase := float64(t) < float64(period)*duty

	// Вычисляем напряжение в зависимости от того, находимся ли мы внутри активной фазы меандра
	var voltage float64
	if inActivePhase {
		voltage = pwm.maxVoltage
	} else {
		voltage = pwm.minVoltage
	}

	return voltage
}

// Метод  Simulate выполняет моделирование работы мотора.
// Данный метод будет имитировать работу мотора управляемого ШИМ-сигналом
// в течение определенного времени с определенным шагом.
// func (m *dcMotor) Simulate() error
// error - ошибка, возникающая при некорректных входных параметрах или при выполнении операций в методе.
func (s *Simulation) Simulate() {
	s.lock.Lock()
	defer s.lock.Unlock()

	ave := AverageValues{}

	// Рассчитываем количество шагов, которые нужно выполнить
	aveNumSteps := s.simtime.duration / s.simtime.step
	aveStepTime := s.simtime.step

	const curNumSteps = 10
	curStepTime := s.simtime.step / curNumSteps

	//time.Duration(j) * s.simtime.duration
	sumPwmVoltage := 0.0
	sumPwmCurrent := 0.0
	sumSpeed := 0.0
	sumVoltage := 0.0
	sumPower := 0.0
	sumCurrent := 0.0

	for i := int64(0); i < aveNumSteps; i++ {
		// Инициализируем переменные для суммирования значений на каждом шаге
		curPwmVoltage := 0.0
		curPwmCurrent := 0.0
		curSpeed := 0.0
		curVoltage := 0.0
		curPower := 0.0
		curCurrent := 0.0

		// Запускаем цикл, который будет выполняться в течение указанного количества шагов
		for j := int64(0); j < curNumSteps; j++ {
			// Создаем структуру для хранения текущих значений на каждом шаге
			cur := currentValues{}
			// Вычисляем текущее время
			currentTime := i*aveStepTime + j*curStepTime
			// Вычисляем сигнал ШИМ на текущем шаге
			cur.pwmVoltage = s.pwm.signal(currentTime)
			// Вычисляем напряжение на двигателе на текущем шаге
			cur.voltage = s.voltage(cur.pwmVoltage)
			// Вычисляем ток на двигателе на текущем шаге
			cur.current = s.current(cur.pwmVoltage, s.nominal.statorResistance)
			// Вычисляем мощность на двигателе на текущем шаге
			cur.power = s.power(cur.voltage, cur.current)
			// Вычисляем скорость вращения двигателя на текущем шаге
			cur.speed = s.speed(cur.power)
			// Обновляем текущие значения
			cur.stepIndex = int(j)
			// Добавляем текущие значения в сумму значений на каждом шаге
			curPwmVoltage += cur.pwmVoltage
			curPwmCurrent += cur.current
			curVoltage += cur.voltage
			curPower += cur.power
			curCurrent += cur.current
			curSpeed += cur.speed
			// Добавляем текущие значения в список значений текущего шага
			s.currval = append(s.currval, cur)
		}

		// Вычисляем средние значения на каждом шаге
		ave.PWMVoltage = curPwmVoltage / float64(curNumSteps)
		ave.PWMCurrent = curPwmCurrent / float64(curNumSteps)
		ave.Voltage = curVoltage / float64(curNumSteps)
		ave.Power = curPower / float64(curNumSteps)
		ave.Current = curCurrent / float64(curNumSteps)
		ave.Speed = curVoltage / float64(curNumSteps)

		sumPwmVoltage += curPwmVoltage
		sumPwmCurrent += curCurrent
		sumVoltage += curVoltage
		sumPower += curPower
		sumCurrent += curCurrent
		sumSpeed += curSpeed

		s.AverVal = append(s.AverVal, ave)
	}

	// Записываем средние значения в структуру Simulation
	s.RestVal.pwmVoltage = sumPwmVoltage
	s.RestVal.pwmCurrent = sumPwmCurrent
	s.RestVal.voltage = sumVoltage
	s.RestVal.power = sumPower
	s.RestVal.current = sumCurrent
	s.RestVal.speed = sumSpeed
}

func (s *Simulation) voltage(v float64) float64 {
	return v
}

func (s *Simulation) current(v float64, r float64) float64 {
	return v / r
}

func (s *Simulation) power(v float64, i float64) float64 {
	return v * i
}

func (s *Simulation) speed(p float64) float64 {
	return p
}

// Тип обработчика полученного результата симуляции в SaveResult
type OutputHandler func(filename string, sim *Simulation) error

// Метод используется для сохранения результата симуляции в файле соответствующего формате
func (s *Simulation) SaveResult(out OutputHandler, filename string) error {
	// (config.Config).Filename
	// s.average
	// параметры - название и требуемы формат файла (csv или svg)
	// обработка ошибок
	// ...
	// вызов хэндлера выполняющего обработку и запись результатов симуляции в файл соответсвующего формата,
	// название файла берется из структуры config.Config и передается хэндлеру аргументом
	// ...
	return nil
}

// Доступные хэндлеры:

// 1. svgout.Output записывает  результаты симуляции m.Results в SVG-файл
// Сигнатура: func Output(filename string, sim Simulation) error
// Описание: Функция реализуется в пакете output/svgout - отвечающем за сохранение результатов симуляции в виде графиков в SVG-формате
// тело пакета и функции нужно сгенерировать
// 2. csvout.Output записывает  результаты симуляции m.Results в CSV-файл
// Сигнатура: func Output(filename string, sim Simulation) error
// Описание: Функция реализуется в пакете output/csvout - отвечающем за запись результатов симуляции в CSV-файл.
// Тело пакета и функции нужно сгенерировать

// Таймер для измерения времени симуляции
type Timer struct {
	startTime time.Time
	endTime   time.Time
}

// Метод для начала отсчета времени
func TimerStart() *Timer {
	ts := time.Now()
	return &Timer{ts, ts}
}

// Метод для окончания отсчета времени
func (t *Timer) TimerStop() time.Duration {
	t.endTime = time.Now()
	return time.Duration(t.endTime.Sub(t.startTime).Microseconds())
}
