package pwmgen

import (
	"fmt"
	"math"

	"github.com/pushking812/motorSim/config"
)

// Структура пакета simulation: параметры ШИМ-генератора.
type PWMGenerator struct {
	minVoltage float64 // Напряжение неактивной фазы
	maxVoltage float64 // Напряжение активной фазы ШИМ-генератора
	duty       float64 // Скважность ШИМ-генератора, в процентах от 0 до 100
	frequency  int64   // Частота ШИМ-генератора, в Hz от 0 до 1000
}

type signalPoint struct {
	leftSide  float64
	point     float64
	rightSide float64
}

// Конструктор пакета simulation: инициализация параметров ШИМ-генератора
func NewPWMGenerator(cfg *config.Config) *PWMGenerator {
	return &PWMGenerator{
		minVoltage: cfg.PWMConfigs[cfg.PWMConfigDefault].MinVoltage,
		maxVoltage: cfg.PWMConfigs[cfg.PWMConfigDefault].MaxVoltage,
		duty:       cfg.PWMConfigs[cfg.PWMConfigDefault].Duty,
		frequency:  cfg.PWMConfigs[cfg.PWMConfigDefault].Frequency,
	}
}

// Пакет simulation: Метод моделирует работу PWM-генератора сигнала в виде меандра.
// Метод возвращает напряжение PWM-генератора в заданный параметром elapsed
// момент времени с начала моделирования в секундах. Для вычисления результата
// использует параметры генератора: минимальное и максимальное напряжение, скважность
// и частоту
func (pwm *PWMGenerator) signal(elapsed float64) float64 {
	const T = 1 // ед. изм. времени
	// Вычисляем период меандра в микросекундах
	period := T / float64(pwm.frequency)

	// Вычисляем скважность меандра в долях единицы (от 0 до 1)
	duty := float64(pwm.duty) / 100

	// Вычисляем время, прошедшее с начала текущего периода меандра
	t := math.Mod(elapsed, period)

	// Определяем, находится ли текущий момент времени внутри активной фазы меандра:
	dl := 0.999999999
	//dr := 1.000000001
	inActivePhaseLeft := (t * dl) < period*duty
	inActivePhasePoint := t < period*duty
	//inActivePhaseRight := (t * dr) < period*duty

	// Вычисляем напряжение в зависимости от того, находимся ли мы внутри активной фазы меандра
	var voltage float64

	if (inActivePhasePoint && inActivePhaseLeft) || (!inActivePhasePoint && inActivePhaseLeft) {
		voltage = pwm.maxVoltage
	} else {
		voltage = pwm.minVoltage
	}

	fmt.Printf("signal: elapsed=%f frequency=%d period=%f duty=%f t=%f active phase point?=%t active phase left?=%t V=%f\n",
		elapsed, pwm.frequency, period, duty, t, inActivePhasePoint, inActivePhaseLeft, voltage)

	return voltage
}
