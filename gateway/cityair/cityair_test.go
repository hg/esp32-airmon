package cityair

import (
	"testing"
	"time"
)

func TestToFloat(t *testing.T) {
	tests := []struct {
		name string
		in   any
		want float32
		ok   bool
	}{
		{name: "float32", in: float32(1.5), want: float32(1.5), ok: true},
		{name: "float64", in: float64(2.25), want: float32(2.25), ok: true},
		{name: "int", in: int(3), want: float32(3), ok: true},
		{name: "int32", in: int32(-4), want: float32(-4), ok: true},
		{name: "int64", in: int64(5), want: float32(5), ok: true},
		{name: "string", in: "123", ok: false},
		{name: "nil", in: nil, ok: false},
		{name: "bool", in: true, ok: false},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			got, ok := toFloat(tt.in)
			if ok != tt.ok {
				t.Fatalf("ok = %v, want %v", ok, tt.ok)
			}
			if ok && got != tt.want {
				t.Fatalf("got %v, want %v", got, tt.want)
			}
		})
	}
}

func TestParseDate(t *testing.T) {
	tests := []struct {
		name  string
		input any
		want  time.Time
		error bool
	}{
		{
			name:  "valid date",
			input: "2024-01-02T15:04:05Z",
			want:  time.Date(2024, 1, 2, 15, 4, 5, 0, time.UTC),
			error: false,
		},
		{name: "invalid format", input: "2024-01-02 15:04:05", error: true},
		{name: "empty string", input: "", error: true},
		{name: "non-string input", input: 123, error: true},
		{name: "nil input", input: nil, error: true},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			got, err := parseDate(tt.input)
			if tt.error {
				if err == nil {
					t.Fatalf("expected error, got nil")
				}
				return
			}
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if !got.Equal(tt.want) {
				t.Fatalf("got %v, want %v", got, tt.want)
			}
		})
	}
}
