// Пакет реализует симуляцию работы мотора управляемого PWM-генератором

package simulation

import (
	"fmt"
	"math"
	"reflect"
	"testing"
)

func TestPWMGeneratorSignal(t *testing.T) {
	s := &Simulation{
		pwm: &pwmGenerator{
			minVoltage: 0,
			maxVoltage: 5,
			duty:       50,
			frequency:  1000,
		},
		simtime: &simTime{duration: 0.005},
	}

	// вычисляет временные параметры моделирования, возвращает количество шагов
	ns := s.calcSimTime()
	st := s.simtime.step // количество шагов моделирования
	fmt.Printf("test: ns=%d, st=%f\n", ns, st)

	elapsed := make([]float64, ns)
	expected := make([]int64, ns)
	for i := range elapsed {
		elapsed[i] = st * float64(i)
		expected[i] = 5

		if (i+1)%4 == 0 {
			expected[i] = 0
		}
	}

	fmt.Printf("test: elapsed=%.6f\n", elapsed)
	fmt.Printf("test: expected=%d\n", expected)

	//expected := []float64{5, 5, 5, 0, 5, 5, 5, 0, 5, 5, 5}

	for i, v := range elapsed {
		if got := s.pwm.signal(v); got != float64(expected[i]) {
			t.Errorf("pwm.signal(%v) = %v, want %v", v, got, expected[i])
		}
	}
}

func TestSimulate(t *testing.T) {
	// инициализация параметров симуляции
	motor := &dcMotor{voltage: 12, power: 12, current: 1, statorResistance: 10,
		emConstant: 1.2, statorInductance: 0.001, efficiency: 0.9, maxCurrent: 30,
		maxSpeed: 10, torqueConstant: 0.01, loadTorque: 0.05}

	env := &environment{temperature: 25, humidity: 50, pressure: 760}

	pwm := &pwmGenerator{minVoltage: 0, maxVoltage: 5, duty: 50, frequency: 1000}

	st := &simTime{duration: 0.005}

	sim := &Simulation{nominal: motor, env: env, pwm: pwm, simtime: st,
		currval: []currentValues{}, AverVal: []AverageValues{}, RestVal: &ResultValues{}}

	// вызов симуляции
	sim.Simulate()

	// проверка результатов
	expectedCurrval := []currentValues{
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 0},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 1},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 2},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 3},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 4},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 5},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 6},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 7},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 8},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 9},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 0},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 1},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 2},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 3},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 4},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 5},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 6},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 7},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 8},
		{pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, stepIndex: 9},
	}
	if !reflect.DeepEqual(sim.currval, expectedCurrval) {
		t.Errorf("unexpected currval: got %v, expected %v", sim.currval, expectedCurrval)
	}

	expectedAverVal := []AverageValues{
		{PWMVoltage: 5, PWMCurrent: 1, Speed: 100, Voltage: 12, Power: 5, Current: 2, Torque: 0.001, StepIndex: 0},
		{PWMVoltage: 5, PWMCurrent: 1, Speed: 100, Voltage: 12, Power: 5, Current: 2, Torque: 0.001, StepIndex: 1},
	}
	if !reflect.DeepEqual(sim.AverVal, expectedAverVal) {
		t.Errorf("unexpected averVal: got %v, expected %v", sim.AverVal, expectedAverVal)
	}

	expectedRestVal := &ResultValues{
		pwmVoltage: 5, pwmCurrent: 1, speed: 100, voltage: 12, power: 5, current: 2, torque: 0.001, duration: st.duration,
	}
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

func TestCalcSpeed(t *testing.T) {
	s := Simulation{
		simtime: &simTime{
			duration: 0.05,
			step:     0.01,
		},
		nominal: &dcMotor{
			maxSpeed:         837.7580409572781 / (2 * math.Pi),
			statorResistance: 2.2,
		},
	}

	const c1 = 0.0037
	const c2 = 0.0
	const c3 = 0.047
	const c4 = 0.0000079
	const inertia = 0.1
	const voltage = 5.0

	w := s.nominal.maxSpeed * 2 * math.Pi
	i := voltage / s.nominal.statorResistance

	for t := 0.0; t < s.simtime.duration; t += s.simtime.step {
		P := c1*w + c2*w*w
		T := c3*i - c4*w*i
		dw := (P - T*w) / inertia
		w += dw * s.simtime.step
	}

	expected := w

	result := s.calcSpeed(voltage)
	//expected := 837.7580409572781
	if !almostEqual(result, expected) {
		t.Errorf("unexpected result: %v, expected: %v", result, expected)
	}
}

func almostEqual(a, b float64) bool {
	epsilon := 0.000001
	return math.Abs(a-b) < epsilon
}

func TestCalcTorque(t *testing.T) {
	m := &dcMotor{
		maxCurrent:       1.0,
		current:          0.5,
		torqueConstant:   1.0,
		statorResistance: 2.2,
	}

	const c1 = 0.03
	const c2 = 0.03
	const c3 = 0.015
	const c4 = 0.00005

	const steptime = 0.01

	expected :=  c1*steptime + c2*math.Pow(steptime, 2) + c3*m.current + c4*m.current*steptime

	result := m.calcTorque(steptime)
	//expected := 0.0315
	if !almostEqual(result, expected) {
		t.Errorf("unexpected result: %v, expected: %v", result, expected)
	}
}
