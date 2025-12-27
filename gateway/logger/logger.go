package logger

import (
	"sync"

	"go.uber.org/zap"
)

type System string

var (
	mu      = sync.Mutex{}
	loggers = make(map[System]*zap.Logger)
)

const (
	Main        System = "main"
	Airkaz      System = "airkaz"
	Influx      System = "influx"
	Kazhydromet System = "kazhydromet"
	Time        System = "time"
	Mqtt        System = "mqtt"
	Net         System = "net"
	Storage     System = "storage"
	CityAir     System = "cityair"
)

func Get(name System) *zap.Logger {
	if logger, ok := loggers[name]; ok {
		return logger
	}

	mu.Lock()
	defer mu.Unlock()

	if logger, ok := loggers[name]; ok {
		return logger
	}

	logger, _ := zap.NewProduction()
	loggers[name] = logger.Named(string(name))
	return logger
}
