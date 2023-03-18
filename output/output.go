package output

import "github.com/pushking812/motorsim/simulation"

type Outputer interface {
	SaveOutput(filename string, results *simulation.Simulation) error
}

type Resulter interface {
	GetResult()
}

//
type Output struct {
	res   Resulter
	out   Outputer
	fname string
}

//
type Result struct {
}

func (o *Result) GetResult() {
}

//
func NewOutput(r Resulter, o Outputer, f string) *Output {
	return &Output{
		out:   o,
		res:   r,
		fname: f,
	}
}

// Метод используется для сохранения результата моделирования в файле соответствующего формате
func (o *Output) SaveResult() error {
	// (config.Config).Filename
	// s.average
	// параметры - название и требуемы формат файла (csv или svg)
	// обработка ошибок
	// ...
	// вызов хэндлера выполняющего обработку и запись результатов моделирования в файл соответсвующего формата,
	// название файла берется из структуры config.Config и передается хэндлеру аргументом
	// ...
	return nil
}

// Тип обработчика полученного результата моделирования в SaveResult
//type OutputHandler func(filename string, sim *simulation.Simulation) error

// Доступные хэндлеры:

// 1. svgout.Output записывает  результаты моделирования m.Results в SVG-файл
// Сигнатура: func Output(filename string, sim Simulation) error
// Описание: Функция реализуется в пакете output/svgout - отвечающем за сохранение результатов моделирования в виде графиков в SVG-формате
// тело пакета и функции нужно сгенерировать
// 2. csvout.Output записывает  результаты моделирования m.Results в CSV-файл
// Сигнатура: func Output(filename string, sim Simulation) error
// Описание: Функция реализуется в пакете output/csvout - отвечающем за запись результатов моделирования в CSV-файл.
// Тело пакета и функции нужно сгенерировать
