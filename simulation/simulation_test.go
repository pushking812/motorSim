// Пакет реализует симуляцию работы мотора управляемого PWM-генератором

package simulation

import (
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

func TestSimulation_Simulate(t *testing.T) {
	tests := []struct {
		name string
		s    *Simulation
	}{
		// TODO: Add test cases.
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			tt.s.Simulate()
		})
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

func TestDcMotorCalcSpeed(t *testing.T) {
	motor := &dcMotor{emConstant: 0.5, maxSpeed: 1000, statorResistance: 5, efficiency: 0.8, torqueConstant: 0.1, maxCurrent: 10, voltage: 12, loadTorque: 0}
	power := 50.0
	expected := 333.3333333333333
	if output := motor.calcSpeed(power); output != expected {
		t.Errorf("calcSpeed(%v) = %v, expected %v", power, output, expected)
	}
}

func TestDcMotorCalcTorque(t *testing.T) {
	motor := &dcMotor{emConstant: 0.5, maxSpeed: 1000, statorResistance: 5, efficiency: 0.8, torqueConstant: 0.1, maxCurrent: 10, voltage: 12, loadTorque: 0}
	speed := 500.0
	expected := 0.09142857142857143
	if output := motor.calcTorque(speed); output != expected {
		t.Errorf("calcTorque(%v) = %v, expected %v", speed, output, expected)
	}
}
