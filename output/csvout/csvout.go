package csvout

import (
	"encoding/csv"
	"os"
	"strconv"

	"github.com/pushking812/motorSim/simulation"
)

// Output записывает результаты симуляции в CSV-файл
func Output(filename string, results *simulation.Simulation) error {
	file, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()

	// Записываем заголовки колонок в CSV-файл
	header := []string{"time", "speed", "torque", "voltage", "current"}
	writer.Write(header)

	// Записываем данные симуляции в CSV-файл
	for i, point := range results.GetAveValues() {
		record := []string{
			strconv.FormatFloat(float64(i), 'f', -1, 64),
			strconv.FormatFloat(point.Speed, 'f', -1, 64),
			strconv.FormatFloat(point.Power, 'f', -1, 64),
			strconv.FormatFloat(point.Voltage, 'f', -1, 64),
			strconv.FormatFloat(point.Current, 'f', -1, 64),
		}
		err := writer.Write(record)
		if err != nil {
			return err
		}
	}

	return nil
}
