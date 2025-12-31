package data

import (
	"testing"
)

func TestNormalizeLocality(t *testing.T) {
	tests := []struct {
		input, want string
	}{
		{" 	Beijing ", "Beijing"},
		{" 東京都\t", "東京都"},
		{"Арал", "Арал"},
		{"поселок Кызыл-Сай", "Кызыл-Сай"},
		{"Поселок Дамба", "Дамба"},
		{"село Косшагыл", "Косшагыл"},
		{"Атырау", "Атырау"},
		{"с.Сарыозек", "Сарыозек"},
		{"п. Береке", "Береке"},
		{"СЕЛО Косшагыл ", "Косшагыл"},
		{" Кольсайские    озера ", "Кольсайские озера"},
		{"  поселк 	Бестобе", "Бестобе"},
		{"Посёлок Торетам ", "Торетам"},
	}
	for _, tt := range tests {
		t.Run(tt.input, func(t *testing.T) {
			got := NormalizeLocality(tt.input)
			if got != tt.want {
				t.Errorf("got %q, want %q", got, tt.want)
			}
		})
	}
}
