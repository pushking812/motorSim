package config

import (
	"reflect"
	"testing"
)

func TestLoadConfig(t *testing.T) {
	configFile := "config.json"
	expectedOutputFile := "output.csv"
	expectedMotorMode := "debug"
	expectedSimDefault := "default"
	expectedPWMDefault := "default"
	expectedEnvDefault := "default"
	expectedMotDefault := "default"
	expectedSimConfig := map[string]SimConfig{
		"default": {
			Duration: 10,
			Step:     50,
		},
		"fast": {
			Duration: 5,
			Step:     10,
		},
	}
	expectedPWMConfig := map[string]PWMConfig{
		"default": {
			Name:       "default",
			MinVoltage: 0,
			MaxVoltage: 5,
			Duty:       0.5,
			Frequency:  50,
		},
	}
	expectedMotConfig := map[string]MotConfig{
		"default": {
			Name:             "default",
			Voltage:          24,
			Power:            1000,
			Current:          20,
			StatorResistance: 0.1,
			EMConstant:       1.2,
			StatorInductance: 0.01,
			Efficiency:       0.9,
			MaxCurrent:       30,
			MaxSpeed:         100,
			TorqueConstant:   0.01,
			LoadTorque:       0.05,
		},
	}
	expectedEnvConfig := map[string]EnvConfig{
		"default": {
			Name:        "default",
			Temperature: 25,
			Humidity:    50,
			Pressure:    760,
		},
	}

	config, err := LoadConfig(configFile)
	if err != nil {
		t.Errorf("LoadConfig returned an error: %v", err)
	}

	if config.OutputFile != expectedOutputFile {
		t.Errorf("LoadConfig returned unexpected output file path: got %v, want %v", config.OutputFile, expectedOutputFile)
	}

	if config.MotorMode != expectedMotorMode {
		t.Errorf("LoadConfig returned unexpected motor mode: got %v, want %v", config.MotorMode, expectedMotorMode)
	}

	if config.SimConfigDefault != expectedSimDefault {
		t.Errorf("LoadConfig returned unexpected sim default: got %v, want %v", config.SimConfigDefault, expectedSimDefault)
	}

	if config.PWMConfigDefault != expectedPWMDefault {
		t.Errorf("LoadConfig returned unexpected PWM default: got %v, want %v", config.PWMConfigDefault, expectedPWMDefault)
	}

	if config.EnvConfigDefault != expectedEnvDefault {
		t.Errorf("LoadConfig returned unexpected env default: got %v, want %v", config.EnvConfigDefault, expectedEnvDefault)
	}

	if config.MotConfigDefault != expectedMotDefault {
		t.Errorf("LoadConfig returned unexpected mot default: got %v, want %v", config.MotConfigDefault, expectedMotDefault)
	}

	if !reflect.DeepEqual(config.SimConfigs, expectedSimConfig) {
		t.Errorf("LoadConfig returned unexpected sim config: got %v, want %v", config.SimConfigs, expectedSimConfig)
	}

	if !reflect.DeepEqual(config.PWMConfigs, expectedPWMConfig) {
		t.Errorf("LoadConfig returned unexpected PWM config: got %v, want %v", config.PWMConfigs, expectedPWMConfig)
	}

	if !reflect.DeepEqual(config.MotConfigs, expectedMotConfig) {
		t.Errorf("LoadConfig returned unexpected mot config: got %v, want %v", config.MotConfigs, expectedMotConfig)
	}
	if !reflect.DeepEqual(config.EnvConfigs, expectedEnvConfig) {
		t.Errorf("LoadConfig returned unexpected env config: got %v, want %v", config.EnvConfigs, expectedEnvConfig)
	}
}
