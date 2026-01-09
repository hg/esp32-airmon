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
	Slug    string
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

type PostID int

type Measure struct {
	PostID PostID
	Rows   []Observation
}
