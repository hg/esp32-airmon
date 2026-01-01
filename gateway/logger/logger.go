package logger

import (
	"log/slog"
	"os"
	"sync"
)

type System string

var mu = sync.Mutex{}
var loggers = make(map[System]*slog.Logger)

const (
	Main        System = "main"
	Airkaz      System = "airkaz"
	Database    System = "database"
	Kazhydromet System = "kazhydromet"
	Mqtt        System = "mqtt"
	Net         System = "net"
	CityAir     System = "cityair"
)

func Get(name System) *slog.Logger {
	if log := loggers[name]; log != nil {
		return log
	}

	mu.Lock()
	defer mu.Unlock()

	log := loggers[name]
	if log == nil {
		log = slog.New(slog.NewJSONHandler(os.Stdout, nil))
		log = log.With("sys", name)
		loggers[name] = log
	}

	return log
}
