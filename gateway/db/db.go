package db

import (
	"context"
	"time"

	"github.com/hg/airmon/data"
	"github.com/hg/airmon/logger"
	"github.com/jackc/pgx/v5/pgxpool"
	"go.uber.org/zap"
)

var log = logger.Get(logger.Database)

type Storage struct {
	ch  chan []data.Measure
	ctx context.Context
	con *pgxpool.Pool
}

func (st *Storage) save(mss []data.Measure) {
	for _, ms := range mss {
		postId, err := st.getPost(ms.Post)
		if err != nil {
			log.Error("unable to get post", zap.Error(err))
			continue
		}
		obsId, err := st.getObservation(postId, ms.Date)
		if err != nil {
			log.Error("unable to get observation", zap.Error(err))
			continue
		}
		for _, lvl := range ms.Level {
			subId, err := st.getSubstance(lvl.Substance)
			if err != nil {
				log.Error("unable to get substance", zap.Error(err))
				continue
			}
			value := data.ConvertUnit(lvl.Value, lvl.Unit)
			st.addLevel(obsId, subId, value)
		}
	}
}

func (st *Storage) receive() {
	for mss := range st.ch {
		start := time.Now()
		st.save(mss)

		spent := time.Since(start)
		log.Info("saved data",
			zap.Int("rows", len(mss)),
			zap.Duration("time", spent))
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
			_ = <-st.ch // drop the oldest measurement and try
		}
	}
}

func NewStorage(settings Settings) (*Storage, error) {
	if err := settings.validate(); err != nil {
		return nil, err
	}

	con, err := pgxpool.New(context.Background(), settings.Uri)
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
