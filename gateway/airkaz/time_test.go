package airkaz

import (
	"encoding/json"
	"testing"
	"time"
)

func TestNoTzMarshalJSON_NilReceiver(t *testing.T) {
	var nt *noTz

	b, err := nt.MarshalJSON()
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	if string(b) != "null" {
		t.Fatalf("want null, got %q", string(b))
	}
}

func TestNoTzMarshalJSON_ValidTime(t *testing.T) {
	loc, _ := time.LoadLocation(tzName)
	ts := time.Date(2024, 1, 2, 3, 4, 5, 0, loc)

	nt := &noTz{Time: ts}

	b, err := nt.MarshalJSON()
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	want := `"2024-01-02 03:04:05"`
	if string(b) != want {
		t.Fatalf("want %q, got %q", want, string(b))
	}
}

func TestNoTzUnmarshalJSON_Null(t *testing.T) {
	var nt noTz

	err := nt.UnmarshalJSON([]byte("null"))
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	if !nt.Time.IsZero() {
		t.Fatalf("want zero time, got %v", nt.Time)
	}
}

func TestNoTzUnmarshalJSON_ValidTime(t *testing.T) {
	var nt noTz

	input := `"2024-01-02 03:04:05"`
	err := nt.UnmarshalJSON([]byte(input))
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	if nt.Location().String() != tzName {
		t.Fatalf("want location %q, got %q", tzName, nt.Location())
	}

	if nt.Year() != 2024 ||
		nt.Month() != time.January ||
		nt.Day() != 2 ||
		nt.Hour() != 3 ||
		nt.Minute() != 4 ||
		nt.Second() != 5 {
		t.Fatalf("unexpected parsed time: %v", nt.Time)
	}
}

func TestNoTzUnmarshalJSON_InvalidFormat(t *testing.T) {
	var nt noTz

	input := `"2024-01-02T03:04:05Z"`
	err := nt.UnmarshalJSON([]byte(input))
	if err == nil {
		t.Fatal("want error, got nil")
	}
}

func TestNoTz_JSONRoundTrip(t *testing.T) {
	loc, _ := time.LoadLocation(tzName)
	original := noTz{
		Time: time.Date(2023, 12, 31, 23, 59, 59, 0, loc),
	}

	b, err := json.Marshal(&original)
	if err != nil {
		t.Fatalf("marshal failed: %v", err)
	}

	var decoded noTz
	if err = json.Unmarshal(b, &decoded); err != nil {
		t.Fatalf("unmarshal failed: %v", err)
	}

	if !original.Time.Equal(decoded.Time) {
		t.Fatalf("want %v, got %v", original.Time, decoded.Time)
	}
}
