package airkaz

import (
	"os"
	"strings"
	"time"
)

const (
	timeFormat = "2006-01-02 15:04:05"
	tzName     = "Asia/Almaty"
)

type noTz struct {
	time.Time
}

var location *time.Location

func init() {
	var err error
	if location, err = time.LoadLocation(tzName); err != nil {
		log.Error("could not find timezone", "timezone", tzName)
		os.Exit(1)
	}
}

func (t *noTz) MarshalJSON() ([]byte, error) {
	if t == nil {
		return []byte("null"), nil
	}
	formatted := `"` + t.Format(timeFormat) + `"`
	return []byte(formatted), nil
}

func (t *noTz) UnmarshalJSON(b []byte) (err error) {
	s := strings.Trim(string(b), `"`)
	if s == "null" {
		t.Time = time.Time{}
		return nil
	}
	t.Time, err = time.ParseInLocation(timeFormat, s, location)
	return err
}
