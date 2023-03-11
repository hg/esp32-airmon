package logger

import (
	"sync"

	"go.uber.org/zap"
)

var (
	mu      = sync.Mutex{}
	loggers = make(map[string]*zap.Logger)
)

const (
	Main        = "main"
	Airkaz      = "airkaz"
	Ceb         = "ceb"
	Influx      = "influx"
	Kazhydromet = "kazhydromet"
	Time        = "time"
	Mqtt        = "mqtt"
	OpenData    = "opendata"
	Net         = "net"
	Storage     = "storage"
)

func Get(name string) *zap.Logger {
	if logger, ok := loggers[name]; ok {
		return logger
	}

	mu.Lock()
	defer mu.Unlock()

	if logger, ok := loggers[name]; ok {
		return logger
	}

	logger, _ := zap.NewProduction()
	loggers[name] = logger.Named(name)
	return logger
}
