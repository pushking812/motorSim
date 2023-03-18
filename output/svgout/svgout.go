package svgout

import "github.com/pushking812/motorsim/simulation"

type SVG struct {
}

// DrawResults рисует графики результатов симуляции и сохраняет их в SVG-файл
func (s *SVG) SaveOutput(filename string, results *simulation.Simulation) error {
	return nil
}
