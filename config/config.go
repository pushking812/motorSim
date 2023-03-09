package config

import (
	"encoding/json"
	"fmt"
	"io/ioutil"
	"reflect"
	"strings"
	"sync"
)

// Структура MotConfig, пакета config: номинальные параметры мотора, загружаются при запуске программы
// из файла конфигурации или командной строки в пакете main
type MotConfig struct {
	Name             string  `json:"name"`       // Название конфигурации
	Voltage          float64 `json:"voltage"`    // Номинальное напряжение, V
	Power            float64 `json:"power"`      // Номинальная мощность, W
	Current          float64 `json:"current"`    // Номинальный ток, A
	StatorResistance float64 `json:"statres"`    // Статорное сопротивление, Ом
	EMConstant       float64 `json:"emc"`        // Электромагнитная постоянная, Вб/А
	StatorInductance float64 `json:"statine"`    // Инерция ротора, кг*м^2
	Efficiency       float64 `json:"efficiency"` // КПД
	MaxCurrent       float64 `json:"maxcurr"`    // Максимальный ток. А
	MaxSpeed         float64 `json:"maxspeed"`   // Максимальная скорость, оборотов/сек
	TorqueConstant   float64 `json:"torque"`     // Момент инерции, кг*м^2
	LoadTorque       float64 `json:"loadtorque"` // Нагрузка, вращающийся объект с моментом сопротивления, Нм

}

// Структура EnvConfig, пакета config: конфигурация параметров окружающей среды, задается при запуске программы
// из файла конфигурации или командной строки в пакете main
type EnvConfig struct {
	Name        string `json:"name"`        // Название конфигурации
	Temperature int    `json:"temperature"` // Температура окружающей среды, °C
	Humidity    int    `json:"humidity"`    // Влажность окружающего воздуха, %
	Pressure    int    `json:"pressure"`    // Атмосферное давление, мм рт. ст.
}

// Структура PWMConfig, пакета config: конфигурация ШИМ-генератора.
type PWMConfig struct {
	Name       string  `json:"name"`        // Название конфигурации
	MinVoltage float64 `json:"min_voltage"` // Напряжение пассивной фазы
	MaxVoltage float64 `json:"max_voltage"` // Напряжение активной фазы ШИМ-генератора
	Duty       float64 `json:"duty"`        // Скважность ШИМ-генератора
	Frequency  int64 `json:"frequency"`   // Частота ШИМ-генератора
}

// Структура SimConfig, пакета config: конфигурация время и шаг симуляции
type SimConfig struct {
	Duration int64 `json:"duration"` // Длительность симуляции, секунды
	Step     int64 `json:"step"`     // Шаг симуляции, миллисекунды
}

// Структура Config, пакета config: конфигурация программы и параметры симуляции
type Config struct {
	OutputFile string `json:"output_file"` // имя файла где будут сохранятся результаты

	MotorMode string `json:"motor_mode"` // Режим работы мотора

	SimConfigs       map[string]SimConfig `json:"sim_config"`  // конфигурации времени и шага симуляции
	PWMConfigs       map[string]PWMConfig `json:"pwm_config"`  // конфигурации PWM-генератора
	MotConfigs       map[string]MotConfig `json:"mot_config"`  // конфигурации определяющие характеристики мотора
	EnvConfigs       map[string]EnvConfig `json:"env_config"`  // конфигурации определяющие условия окружающей среды
	SimConfigDefault string               `json:"sim_default"` // название конфигурации по умолчанию, время и шаг симуляции
	PWMConfigDefault string               `json:"pwm_default"` // название конфигурации по умолчанию, PWM-генератора
	EnvConfigDefault string               `json:"env_default"` // название конфигурации по умолчанию, условия окружающей среды
	MotConfigDefault string               `json:"mot_default"` // название конфигурации по умолчанию, характеристики мотора

	lock sync.RWMutex
}

// Функция пакета config для загрузки конфигурационного файла из JSON
func LoadConfig(filename string) (*Config, error) {
	fmt.Println(filename)
	data, err := ioutil.ReadFile(filename)
	if err != nil {
		return nil, fmt.Errorf("error reading config file: %v", err)
	}

	// Декодирование JSON в структуру Config
	var cfg Config
	cfg.lock.RLock()
	defer cfg.lock.RUnlock()

	err = json.Unmarshal(data, &cfg)
	if err != nil {
		return nil, fmt.Errorf("error decoding config file: %v", err)
	}

	fmt.Printf("\n LoadConfig cfg: %v\n", cfg)

	return &cfg, nil
}

// func (cfg *Config) getValueByName(name string) (interface{}, error) {
// 	fields := strings.Split(name, ".")
// 	var val interface{} = cfg
// 	for _, field := range fields {
// 		var ok bool
// 		val, ok = reflect.ValueOf(val).Elem().FieldByName(strings.Title(field)).Interface(), true
// 		if !ok {
// 			return nil, fmt.Errorf("field %s not found", name)
// 		}
// 	}
// 	return val, nil
// }

// func (cfg *Config) GetInt(name string) (int, error) {
// 	cfg.lock.RLock()
// 	defer cfg.lock.RUnlock()

// 	val, err := cfg.getValueByName(name)
// 	if err != nil {
// 		return 0, err
// 	}

// 	if intVal, ok := val.(int); ok {
// 		return intVal, nil
// 	}
// 	return 0, fmt.Errorf("field %s is not of type int", name)
// }

// func (cfg *Config) GetString(name string) (string, error) {
// 	cfg.lock.RLock()
// 	defer cfg.lock.RUnlock()

// 	val, err := cfg.getValueByName(name)
// 	if err != nil {
// 		return "", err
// 	}

// 	if strVal, ok := val.(string); ok {
// 		return strVal, nil
// 	}
// 	return "", fmt.Errorf("field %s is not of type string", name)
// }

// func (cfg *Config) GetDuration(name string) (time.Duration, error) {
// 	cfg.lock.RLock()
// 	defer cfg.lock.RUnlock()

// 	val, err := cfg.getValueByName(name)
// 	if err != nil {
// 		return 0, err
// 	}

// 	if durVal, ok := val.(time.Duration); ok {
// 		return durVal, nil
// 	}
// 	return 0, fmt.Errorf("field %s is not of type time.Duration", name)
// }

// func (cfg *Config) GetFloat64(name string) (float64, error) {
// 	cfg.lock.RLock()
// 	defer cfg.lock.RUnlock()

// 	val, err := cfg.getValueByName(name)
// 	if err != nil {
// 		return 0, err
// 	}

// 	if floatVal, ok := val.(float64); ok {
// 		return floatVal, nil
// 	}
// 	return 0, fmt.Errorf("field %s is not of type float64", name)
// }

func (cfg *Config) SetParam(name string, value interface{}) error {
	cfg.lock.Lock()
	defer cfg.lock.Unlock()

	fields := strings.Split(name, ".")
	var val reflect.Value = reflect.ValueOf(cfg)
	for _, field := range fields {
		val = reflect.Indirect(val).FieldByName(strings.Title(field))
		if !val.IsValid() {
			return fmt.Errorf("field %s not found", name)
		}
	}

	if !val.CanSet() {
		return fmt.Errorf("field %s is not settable", name)
	}

	valType := val.Type()
	valueType := reflect.TypeOf(value)
	if valType.Kind() != valueType.Kind() {
		return fmt.Errorf("field %s has type %s, but value has type %s", name, valType.Kind(), valueType.Kind())
	}

	val.Set(reflect.ValueOf(value))
	return nil
}

func (c *Config) GetMap(key string, subKey string) map[string]interface{} {
	c.lock.RLock()
	defer c.lock.RUnlock()

	switch key {
	case "sim_config":
		if config, ok := c.SimConfigs[subKey]; ok {
			return map[string]interface{}{
				"duration": config.Duration,
				"step":     config.Step,
			}
		}
	case "pwm_config":
		if config, ok := c.PWMConfigs[subKey]; ok {
			return map[string]interface{}{
				"name":        config.Name,
				"min_voltage": config.MinVoltage,
				"max_voltage": config.MaxVoltage,
				"frequency":   config.Frequency,
				"duty":        config.Duty,
			}
		}
	case "mot_config":
		if config, ok := c.MotConfigs[subKey]; ok {
			return map[string]interface{}{
				"name":       config.Name,
				"voltage":    config.Voltage,
				"power":      config.Power,
				"current":    config.Current,
				"statres":    config.StatorResistance,
				"emc":        config.EMConstant,
				"statine":    config.StatorInductance,
				"efficiency": config.Efficiency,
				"maxcurr":    config.MaxCurrent,
				"maxspeed":   config.MaxSpeed,
				"torque":     config.TorqueConstant,
				"loadtorque": config.LoadTorque,
			}
		}
	case "env_config":
		if config, ok := c.EnvConfigs[subKey]; ok {
			return map[string]interface{}{
				"name":        config.Name,
				"temperature": config.Temperature,
				"humidity":    config.Humidity,
				"pressure":    config.Pressure,
			}
		}
	}
	return nil
}
