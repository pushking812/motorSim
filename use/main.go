package main

import (
	"flag"
	"fmt"
	"os"
	"time"

	"github.com/pushking812/motorSim/config"
	"github.com/pushking812/motorSim/output"
	"github.com/pushking812/motorSim/output/csvout"
	"github.com/pushking812/motorSim/output/svgout"
	"github.com/pushking812/motorSim/simulation"
)

var paramMap = map[string]interface{}{
	"pwm-duty":            50.0,
	"pwm-voltage":         5.0,
	"pwm-frequency":       500.0,
	"motor-config":        "mdc12",
	"mode-config":         "constant",
	"env-config":          "normal",
	"motor-params-config": "speed",
	"duration":            10000,
	"step":                100,
}

func parseArgs() *config.Config {
	// Парсинг аргументов командной строки
	for k, v := range paramMap {
		switch v.(type) {
		case float64:
			paramMap[k] = flag.Float64(k, v.(float64), fmt.Sprintf("%s in percentage", k))
		case string:
			paramMap[k] = flag.String(k, v.(string), fmt.Sprintf("%s configuration name", k))
		case int:
			paramMap[k] = flag.Int(k, v.(int), fmt.Sprintf("simulation %s in milliseconds", k))
		}
	}
	flag.Parse()

	// Чтение параметров из конфигурационного файла
	cfg, err := config.LoadConfig("config.json")
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}

	fmt.Printf("\n parseArgs cfg: %v\n", cfg)

	// Использование аргументов командной строки, если они заданы
	for k, v := range paramMap {
		switch v.(type) {
		case float64:
			if v != paramMap[k].(float64) {
				cfg.SetParam(k, *v.(*float64))
			}
		case string:
			if v != paramMap[k].(string) {
				cfg.SetParam(k, *v.(*string))
			}
		case int:
			if v != paramMap[k].(int) {
				cfg.SetParam(k, time.Duration(*v.(*int))*time.Millisecond)
			}
		}
	}

	return cfg
}

func main() {
	// парсинг конфигурационного файла и параметров командной строки,
	// получение параметров симуляции
	// func parseArgs() *config.Config
	cfg := parseArgs()

	// Инициализация модели симуляции работы мотора
	// func NewSimulation(cfg *config.Config) (*simulation.Simulation, error)
	s, err := simulation.NewSimulation(cfg) //s
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}

	filename, _ := saveResults(&output.Result{}, &csvout.CSV{}, cfg.OutputFile)
	fmt.Println("File converting to CVS: ", filename)

	// type outputHandler func(filename string) error
	// func (s *Simulation) SaveResult(out outputHandler) error
	filename, _ = saveResults(&output.Result{}, &svgout.SVG{}, cfg.OutputFile)

	fmt.Println("File converting to SVG: ", filename)
}

func saveResults(r output.Resulter, o output.Outputer, filename string) (string, error) {
	// Открытие файла для записи результатов
	outfile, err := os.Create(filename)
	if err != nil {
		return "", fmt.Errorf("cannot create output file %s: %v", filename, err)
	}
	defer outfile.Close()

	ofn := outfile.Name()

	// Вызов обработчика для преобразования результата и сохранения результатов в файл определенного формата
	out := output.NewOutput(r, o, ofn)

	err = out.SaveResult()
	if err != nil {
		return "", fmt.Errorf("error saving result to file %s: %v", ofn, err)
	}

	return ofn, nil
}
