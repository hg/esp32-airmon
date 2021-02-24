package storage

import (
	"encoding/json"
	"github.com/hg/airmon/logger"
	"go.uber.org/zap"
	"io/ioutil"
	"os"
	"path"
)

var log = logger.Get(logger.Storage)

func getPath(filename string) (string, error) {
	dir := os.Getenv("XDG_CONFIG_HOME")
	if dir == "" {
		dir = "/var/lib"
	}
	dir = path.Join(dir, "airmon")

	err := os.MkdirAll(dir, 0750)

	if err != nil {
		log.Error("could not create data directory",
			zap.String("path", dir),
			zap.Error(err))

		return "", err
	}

	return path.Join(dir, filename), nil
}

func Save(filename string, data interface{}) error {
	fullPath, err := getPath(filename)
	if err != nil {
		return err
	}

	serialized, err := json.Marshal(data)
	if err != nil {
		log.Error("could not serialize data",
			zap.Any("data", data),
			zap.Error(err))

		return err
	}

	err = ioutil.WriteFile(fullPath, serialized, 0640)
	if err != nil {
		log.Error("could not write file",
			zap.String("path", fullPath),
			zap.ByteString("data", serialized))
	}

	return err
}

func Load(filename string, data interface{}) error {
	fullPath, err := getPath(filename)
	if err != nil {
		return err
	}

	serialized, err := ioutil.ReadFile(fullPath)
	if err != nil {
		log.Error("could not read file", zap.String("path", fullPath))
		return err
	}

	err = json.Unmarshal(serialized, data)
	if err != nil {
		log.Error("could not deserialize data",
			zap.String("path", fullPath),
			zap.ByteString("data", serialized))
	}

	return err
}
