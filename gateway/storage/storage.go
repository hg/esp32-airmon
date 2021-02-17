package storage

import (
	"encoding/json"
	"io/ioutil"
	"os"
	"path"
)

func getPath(filename string) (string, error) {
	dir := os.Getenv("XDG_CONFIG_HOME")
	if dir == "" {
		dir = "/var/lib"
	}
	dir = path.Join(dir, "airmon")

	err := os.MkdirAll(dir, 0750)
	if err != nil {
		return "", err
	}

	return path.Join(dir, filename), nil
}

func Save(filename string, data interface{}) error {
	serialized, err := json.Marshal(data)
	if err != nil {
		return err
	}

	fullPath, err := getPath(filename)
	if err != nil {
		return err
	}

	return ioutil.WriteFile(fullPath, serialized, 0640)
}

func Load(filename string, data interface{}) error {
	fullPath, err := getPath(filename)
	if err != nil {
		return err
	}

	serialized, err := ioutil.ReadFile(fullPath)
	if err != nil {
		return err
	}

	return json.Unmarshal(serialized, data)
}
