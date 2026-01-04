package db

import (
	"context"
	"errors"
	"time"

	"github.com/hg/airmon/data"
	"github.com/hg/airmon/logger"
	"github.com/jackc/pgx/v5/pgxpool"
)

var log = logger.Get(logger.Database)

type Storage struct {
	ch  chan []data.Measure
	ctx context.Context
	con *pgxpool.Pool
}

func (st *Storage) save(mss []data.Measure) {
	// Substances can disappear or be changed at any moment,
	// cache them only for the duration of the batch
	subToID := make(map[string]int)

	for _, ms := range mss {
		postID, err := st.getPost(ms.Post)
		if err != nil {
			log.Error("unable to get post", "error", err)
			continue
		}
		for _, row := range ms.Rows {
			obsID, err := st.getObservation(postID, row.Date)
			if err != nil {
				log.Error("unable to get observation", "error", err)
				continue
			}
			for _, lvl := range row.Level {
				sub := data.NormalizeSubstance(lvl.Substance)
				subID, ok := subToID[sub]
				if !ok {
					subID, err = st.getSubstance(sub)
					if err != nil {
						log.Error("unable to get substance", "error", err)
						continue
					}
					subToID[sub] = subID
				}
				value := data.ConvertUnit(lvl.Value, lvl.Unit)
				st.addLevel(obsID, subID, value)
			}
		}
	}
}

func (st *Storage) receive() {
	for mss := range st.ch {
		start := time.Now()
		st.save(mss)

		if len(mss) > 1 {
			spent := time.Since(start)
			observations, levels := 0, 0
			for _, ms := range mss {
				observations += len(ms.Rows)
				for _, row := range ms.Rows {
					levels += len(row.Level)
				}
			}
			log.Info("saved data",
				"source", mss[0].Post.Source,
				"measures", len(mss),
				"observations", observations,
				"levels", levels,
				"millis", spent.Milliseconds())
		}
	}
}

func (st *Storage) Enqueue(measures []data.Measure) bool {
	if len(measures) == 0 {
		return false
	}

	for try := 0; ; try++ {
		select {
		case st.ch <- measures:
			return true

		case <-time.After(5 * time.Second):
			log.Error("timed out on sending measures")
			if try >= 3 {
				log.Error("unable to send measures, discarding")
				return false
			}
			<-st.ch // drop the oldest measurement and try
		}
	}
}

type Settings struct {
	URI string `json:"uri"`
}

func NewStorage(set Settings) (*Storage, error) {
	if set.URI == "" {
		return nil, errors.New("PostgreSQL URI not set")
	}

	con, err := pgxpool.New(context.Background(), set.URI)
	if err != nil {
		return nil, err
	}

	sender := &Storage{
		ch:  make(chan []data.Measure, 2000),
		ctx: context.Background(),
		con: con,
	}

	go sender.receive()

	return sender, nil
}
