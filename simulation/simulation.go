// Пакет реализует симуляцию работы мотора управляемого PWM-генератором
package simulation

import (
	"errors"
	"sync"
	"time"

	"github.com/pushking812/motorsim/config"
	"github.com/pushking812/motorsim/simulation/dcmotor"
)

type Simulator interface {
	Simulate(filename string, results *Simulation) error
}

// Структура Simulation, пакета simulation: содержит параметры моделирования
type Simulation struct {
	comp []Simulator

	env *environment // параметры моделирования, окружающая среда

	simtime *simTime // параметры моделирования, время и шаг моделирования

	currval []dcmotor.CurrentValues // текущие значения параметров при моделирования, за шаг моделирования
	AverVal []dcmotor.AverageValues // средние значения параметров при моделирования, за период времени
	RestVal *dcmotor.ResultValues   // итоговые значения параметров при моделирования, итоговые за симуляцию

	lock sync.RWMutex
}

// Пакет simulation, структура environment: содержит параметры окружающей среды используюемые
// для расчета текущих значений моделирования, задаются при запуске программы из файла
// конфигурации или командной строки в пакете main
type environment struct {
	temperature int // Температура окружающей среды, °C
	humidity    int // Влажность, %
	pressure    int // Атмосферное давление
}

// время и шаг моделирования
type simTime struct {
	duration float64 // длительность моделирования
	step     float64 // шаг моделирования
}

func (s *Simulation) GetAveValues() []dcmotor.AverageValues {
	return s.AverVal
}

// Конструктор объекта Simulation. Определяет параметры моделирования:
// характеристики мотора, окружающей среды, PWM-генератора
func NewSimulation(cfg *config.Config) (*Simulation, error) {
	if cfg == nil {
		return nil, errors.New("NewSimulation: Can't create object Simulation")
	}

	// Возвращаем указатель на созданный экземпляр моделирования
	return &Simulation{
		comp:    []Simulator{},
		env:     NewEnvironment(cfg),
		simtime: NewSimTime(cfg),
		currval: make([]dcmotor.CurrentValues, 0),
		AverVal: make([]dcmotor.AverageValues, 0),
		RestVal: &dcmotor.ResultValues{},
	}, nil
}

func (s *Simulation) AddComponent(c Simulator) {

}

func (s *Simulation) Run() {

}

//motor *dcmotor.DCMotor     // параметры моделирования, мотор
//pwm   *pwmgen.PWMGenerator // параметры моделирования, PWM-генератор
//motor:   dcmotor.NewDCMotor(cfg),
//pwm:     pwmgen.NewPWMGenerator(cfg),

// Пакет simulation: инициализация параметров окружающей среды
func NewEnvironment(cfg *config.Config) *environment {
	return &environment{
		temperature: cfg.EnvConfigs[cfg.EnvConfigDefault].Temperature,
		humidity:    cfg.EnvConfigs[cfg.EnvConfigDefault].Humidity,
		pressure:    cfg.EnvConfigs[cfg.EnvConfigDefault].Pressure,
	}
}

// время и шаг моделирования
func NewSimTime(cfg *config.Config) *simTime {
	return &simTime{
		duration: cfg.SimConfigs[cfg.SimConfigDefault].Duration,
		step:     cfg.SimConfigs[cfg.SimConfigDefault].Step,
	}
}

// Таймер для измерения времени моделирования
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
