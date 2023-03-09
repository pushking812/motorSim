module github.com/pushking812/motorSim/use

go 1.20

replace github.com/pushking812/motorSim/config => ../config

replace github.com/pushking812/motorSim/simulation => ../simulation

replace github.com/pushking812/motorSim/output => ../output

require (
	github.com/pushking812/motorSim/config v0.0.0-00010101000000-000000000000
	github.com/pushking812/motorSim/output v0.0.0-00010101000000-000000000000
	github.com/pushking812/motorSim/simulation v0.0.0-00010101000000-000000000000
)
