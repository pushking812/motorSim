package main

import (
	"io/ioutil"
	"os"
	"reflect"
	"testing"
)

func TestParseArgs(t *testing.T) {
	// Create a temporary config file for testing
	configFile, err := ioutil.TempFile("", "config.json")
	if err != nil {
		t.Fatalf("error creating temp file: %v", err)
	}
	defer os.Remove(configFile.Name())

	// Write the config file contents to the temporary file
	configData := []byte(`{
		"output_file": "output.csv",
		"motor_mode": "debug",
		"sim_config": {
		  "default": {
			"duration": 10,
			"step": 50
		  },
		  "fast": {
			"duration": 5,
			"step": 10
		  }
		},
		"pwm_config": {
		  "default": {
			"name": "default",
			"min_voltage": 0,
			"max_voltage": 5,
			"duty": 0.5,
			"frequency": 50
		  }
		},
		"mot_config": {
		  "default": {
			"name": "default",
			"voltage": 24,
			"power": 1000,
			"current": 20,
			"statres": 0.1,
			"emc": 1.2,
			"statine": 0.01,
			"efficiency": 0.9,
			"maxcurr": 30,
			"maxspeed": 100,
			"torque": 0.01,
			"loadtorque": 0.05
		  }
		},
		"env_config": {
		  "default": {
			"name": "default",
			"temperature": 25,
			"humidity": 50,
			"pressure": 760
		  }
		},
		"sim_default": "default",
		"pwm_default": "default",
		"env_default": "default",
		"mot_default": "default"
	}`)
	if _, err := configFile.Write(configData); err != nil {
		t.Fatalf("error writing to temp file: %v", err)
	}

	// Call the parseArgs function with the test arguments
	cfg := parseArgs()

	// Check that the configuration was loaded correctly
	expectedOutputFile := "output.csv"
	if cfg.OutputFile != expectedOutputFile {
		t.Errorf("expected output_file to be %q, but got %q", expectedOutputFile, cfg.OutputFile)
	}

	expectedMotorMode := "debug"
	if cfg.MotorMode != expectedMotorMode {
		t.Errorf("expected motor_mode to be %q, but got %q", expectedMotorMode, cfg.MotorMode)
	}

	expectedSimConfig := map[string]interface{}{
		"duration": int64(5),
		"step":     int64(10),
	}
	if !reflect.DeepEqual(cfg.GetMap("sim_config", "fast"), expectedSimConfig) {
		t.Errorf("expected sim_config.fast to be %v, but got %v", expectedSimConfig, cfg.GetMap("sim_config", "fast"))

		expectedPwmConfig := map[string]interface{}{
			"name":        "default",
			"min_voltage": float64(0.0),
			"max_voltage": float64(5.0),
			"duty":        float64(0.5),
			"frequency":   int64(50),
		}
		if !reflect.DeepEqual(cfg.GetMap("pwm_config", "default"), expectedPwmConfig) {
			t.Errorf("expected pwm_config.default to be %v, but got %v", expectedPwmConfig, cfg.GetMap("pwm_config", "default"))
		}

		expectedMotConfig := map[string]interface{}{
			"name":       "default",
			"voltage":    float64(24),
			"power":      float64(1000),
			"current":    float64(20),
			"statres":    float64(0.1),
			"emc":        float64(1.2),
			"statine":    float64(0.01),
			"efficiency": float64(0.9),
			"maxcurr":    float64(30),
			"maxspeed":   float64(100),
			"torque":     float64(0.01),
			"loadtorque": float64(0.05),
		}
		if !reflect.DeepEqual(cfg.GetMap("mot_config", "default"), expectedMotConfig) {
			t.Errorf("expected mot_config.default to be %v, but got %v", expectedMotConfig, cfg.GetMap("mot_config", "default"))
		}

		expectedEnvConfig := map[string]interface{}{
			"name":        "default",
			"temperature": float64(25),
			"humidity":    float64(50),
			"pressure":    float64(760),
		}
		if !reflect.DeepEqual(cfg.GetMap("env_config", "default"), expectedEnvConfig) {
			t.Errorf("expected env_config.default to be %v, but got %v", expectedEnvConfig, cfg.GetMap("env_config", "default"))
		}
	}
}
