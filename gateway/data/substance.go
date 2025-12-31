package data

import "strings"

func ConvertUnit(value float32, from string) float32 {
	from = strings.ToUpper(from)

	// TODO: support more best-effort conversions
	if from == "MG/M3" || from == "MG/M³" {
		return value * 1000
	}

	return value
}

func NormalizeSubstance(name string) string {
	up := strings.ToUpper(strings.TrimSpace(name))

	switch up {
	case "HUMIDITY":
		return "RH"
	case "PRESSURE":
		return "BP"
	case "PM2":
		return "PM2.5"
	case "CH2O":
		return "HCOH"
	case "AT", "TEMPC", "TEMPERATURE":
		return "TEMP"
	case "WVA":
		return "WS"
	case "WDA":
		return "WD"
	default:
		return up
	}
}
