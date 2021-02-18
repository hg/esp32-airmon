package tm

import (
	"log"
	"strings"
	"time"
)

const timeFormat = "2006-01-02 15:04:05"

type Time struct {
	time.Time
}

var location *time.Location

func init() {
	var err error
	if location, err = time.LoadLocation("Asia/Almaty"); err != nil {
		log.Fatal("could not find timezone")
	}
}

func (t *Time) MarshalJSON() ([]byte, error) {
	if t == nil {
		return []byte("null"), nil
	}
	formatted := t.Time.Format(timeFormat)
	return []byte(formatted), nil
}

func (t *Time) UnmarshalJSON(b []byte) (err error) {
	s := strings.Trim(string(b), `"`)
	if s == "null" {
		t.Time = time.Time{}
		return nil
	}
	t.Time, err = time.ParseInLocation(timeFormat, s, location)
	return err
}
