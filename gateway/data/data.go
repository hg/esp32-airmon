package data

import (
	"time"

	"github.com/hg/airmon/spatial"
)

type Source string

const (
	Airkaz      Source = "airkaz"
	CityAir     Source = "cityair"
	Custom      Source = "custom"
	Kazhydromet Source = "kazhydromet"
)

type Post struct {
	Source  Source
	Name    string
	City    string
	Address string
	Geo     spatial.Point
}

type Level struct {
	Substance string
	Unit      string
	Value     float32
}

type Observation struct {
	Date  time.Time
	Level []Level
}

type Measure struct {
	Post Post
	Rows []Observation
}
