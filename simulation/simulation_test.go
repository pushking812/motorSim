// Пакет реализует симуляцию работы мотора управляемого PWM-генератором

package simulation

import "testing"

func TestPWMGeneratorSignal(t *testing.T) {
	pwm := pwmGenerator{
		minVoltage: 0,
		maxVoltage: 5,
		duty:       50,
		frequency:  100,
	}

	elapsed:=[]int64{3,5,7,10,12,15,17,20,23,25}// 10 milliseconds into the simulation
	expected := []float64{5,5,0,5,5,5,0,5,5,5}// voltage should be at maxVoltage

	for i, v := range elapsed {
		if got := pwm.signal(v); got != expected[i] {
			t.Errorf("pwm.signal(%v) = %v, want %v", v, got, expected[i])
		}
	}
}
