package data

import "time"

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
	Lon     float32
	Lat     float32
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
