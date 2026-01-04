package data

import (
	"fmt"
	"testing"
)

func TestConvertUnit(t *testing.T) {
	tests := []struct {
		val  float32
		unit string
		want float32
	}{
		{val: 12.34, unit: "unknown", want: 12.34},
		{val: 5.3, unit: "mg/m3", want: 5300},
		{val: 0.15, unit: "MG/M³", want: 150},
		{val: 2.25, unit: "mcg/m3", want: 2.25},
		{val: 4.1, unit: "m/s", want: 4.1},
	}
	for _, tt := range tests {
		name := fmt.Sprintf("%f %s", tt.val, tt.unit)
		t.Run(name, func(t *testing.T) {
			if got := ConvertUnit(tt.val, tt.unit); got != tt.want {
				t.Errorf("ConvertUnit() = %v, want %v", got, tt.want)
			}
		})
	}
}

func TestNormalizeSubstance(t *testing.T) {
	tests := []struct {
		in, want string
	}{
		{in: "", want: ""},
		{in: " \t\r\n ", want: ""},
		{in: "	ps -auxf @)(#*&	", want: "PS -AUXF @)(#*&"},
		{in: "humidity", want: "RH"},
		{in: "PRESSURE", want: "BP"},
		{in: "tempC", want: "TEMP"},
		{in: "quux", want: "QUUX"},
	}
	for _, tt := range tests {
		t.Run(tt.in, func(t *testing.T) {
			if got := NormalizeSubstance(tt.in); got != tt.want {
				t.Errorf("NormalizeSubstance() = %v, want %v", got, tt.want)
			}
		})
	}
}
