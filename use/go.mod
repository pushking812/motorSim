module github.com/pushking812/motorsim/use

go 1.20

replace github.com/pushking812/motorsim/config => ../config

replace github.com/pushking812/motorsim/simulation => ../simulation

replace github.com/pushking812/motorsim/output => ../output

require (
	github.com/pushking812/motorsim/config v0.0.0-00010101000000-000000000000
	github.com/pushking812/motorsim/output v0.0.0-00010101000000-000000000000
	github.com/pushking812/motorsim/simulation v0.0.0-00010101000000-000000000000
)
