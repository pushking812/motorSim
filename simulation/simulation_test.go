// Пакет реализует симуляцию работы мотора управляемого PWM-генератором

package simulation

import (
	"fmt"
	"reflect"
	"testing"
)

func TestPWMGeneratorSignal(t *testing.T) {
	pwm := pwmGenerator{
		minVoltage: 0,
		maxVoltage: 5,
		duty:       50,
		frequency:  100,
	}

	elapsed := []int64{3, 5, 7, 10, 12, 15, 17, 20, 23, 25} // 10 milliseconds into the simulation
	expected := []float64{5, 5, 0, 5, 5, 5, 0, 5, 5, 5}     // voltage should be at maxVoltage

	for i, v := range elapsed {
		if got := pwm.signal(v); got != expected[i] {
			t.Errorf("pwm.signal(%v) = %v, want %v", v, got, expected[i])
		}
	}
}

func TestSimulate(t *testing.T) {
	// инициализация параметров симуляции
	motor := &dcMotor{
		voltage:          12,
		power:            5,
		current:          1,
		statorResistance: 50,
		emConstant:       0.05,
		statorInductance: 0.001,
		efficiency:       0.85,
		maxCurrent:       2,
		maxSpeed:         2000,
		torqueConstant:   0.1,
		loadTorque:       5,
	}
	env := &environment{
		temperature: 25,
		humidity:    50,
		pressure:    760,
	}
	pwm := &pwmGenerator{
		minVoltage: 0,
		maxVoltage: 12,
		duty:       0.5,
		frequency:  1000,
	}
	st := &simTime{
		duration: 1000,
		step:     10,
	}

	sim := &Simulation{
		nominal: motor,
		env:     env,
		pwm:     pwm,
		simtime: st,

		currval: []currentValues{},
		AverVal: []AverageValues{},
		RestVal: &ResultValues{},
	}

	// вызов симуляции
	sim.Simulate()

	// проверка результатов
	expectedCurrval := []currentValues{
		{stepIndex: 0, voltage: 12, current: 20, speed: 0},
		// и т.д, заполнить еще 19 строками
	}
	if !reflect.DeepEqual(sim.currval, expectedCurrval) {
		t.Errorf("unexpected currval: got %v, expected %v", sim.currval, expectedCurrval)
	}

	expectedAverVal := []AverageValues{
		{StepIndex: 0, Current: 20, Speed: 0},
		// и т.д, заполнить еще 1 строкой
	}

	if !reflect.DeepEqual(sim.AverVal, expectedAverVal) {
		t.Errorf("unexpected averVal: got %v, expected %v", sim.AverVal, expectedAverVal)
	}

	expectedRestVal := &ResultValues{duration: /*формула*/ 0, current: 20, speed: 0}

	if !reflect.DeepEqual(sim.RestVal, expectedRestVal) {
		t.Errorf("unexpected averVal: got %v, expected %v", sim.AverVal, expectedAverVal)
	}
}

func TestDcMotorCalcVoltage(t *testing.T) {
	motor := &dcMotor{}
	pwmV := 10.0
	expected := pwmV
	if output := motor.calcVoltage(pwmV); output != expected {
		t.Errorf("calcVoltage(%v) = %v, expected %v", pwmV, output, expected)
	}
}

func TestDcMotorCalcCurrent(t *testing.T) {
	motor := &dcMotor{statorResistance: 5}
	v := 55.0
	expected := v / motor.statorResistance
	if output := motor.calcCurrent(v); output != expected {
		t.Errorf("calcCurrent(%v) = %v, expected %v", v, output, expected)
	}
}

func TestDcMotorCalcPower(t *testing.T) {
	motor := &dcMotor{efficiency: 0.8}
	v := 55.0
	i := 4.0
	expected := v * i * motor.efficiency
	if output := motor.calcPower(v, i); output != expected {
		t.Errorf("calcPower(%v, %v) = %v, expected %v", v, i, output, expected)
	}
}

func TestDcMotorCalcTorque(t *testing.T) {
	motor := &dcMotor{
		torqueConstant: 0.1,
		maxCurrent:     10,
		voltage:        12,
		loadTorque:     0,
	}
	power := 50.0
	expected := 0.417
	if output := motor.calcTorque(power); (expected - output) > 0.001 {
		t.Errorf("calcTorque(%v) = %v, expected %v", power, output, expected)
	}
}

func TestDcMotorCalcSpeed(t *testing.T) {
	motor := &dcMotor{
		emConstant:       0.5,
		maxSpeed:         1000,
		torqueConstant:   0.1,
		maxCurrent:       10,
		voltage:          12,
		loadTorque:       0,
		statorInductance: 0.001,
	}
	power := 50.0
	torque := 0.417
	expected := 958.3

	if output := motor.calcSpeed(power, torque); (expected - output) > 0.001 {
		t.Errorf("calcSpeed(%v) = %v, expected %v", power, output, expected)
	}
	fmt.Printf("power: %f, torque: %f, speed: %f\n", power, torque, expected)
}
