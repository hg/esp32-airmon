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
	Database    System = "database"
	Kazhydromet System = "kazhydromet"
	Time        System = "time"
	Mqtt        System = "mqtt"
	Net         System = "net"
	CityAir     System = "cityair"
)

func Get(name System) *zap.Logger {
	if log, ok := loggers[name]; ok {
		return log
	}

	mu.Lock()
	defer mu.Unlock()

	if log, ok := loggers[name]; ok {
		return log
	}

	log, _ := zap.NewProduction()
	loggers[name] = log.Named(string(name))
	return log
}
