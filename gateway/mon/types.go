package mon

import "github.com/hg/airmon/data"

type DataSource interface {
	Convert() []data.Measure
}
