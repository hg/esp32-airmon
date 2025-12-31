package db

import (
	"errors"
)

type Settings struct {
	Uri string
}

func (se *Settings) validate() error {
	if se.Uri == "" {
		return errors.New("PostgreSQL URI not set")
	}
	return nil
}
