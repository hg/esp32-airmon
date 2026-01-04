package logger

import (
	"testing"
)

func TestGet(t *testing.T) {
	const id1 = "test-logger"
	const id2 = "test-logger-2"

	log1 := Get(id1)
	if log1 == nil {
		t.Fatal("expected non-nil logger")
	}
	log2 := Get(id1)
	if log1 != log2 {
		t.Fatal("expected same logger for same id")
	}
	log3 := Get(id2)
	if log3 == nil {
		t.Fatal("expected non-nil logger for second id")
	}
	if log1 == log3 {
		t.Fatal("expected different logger for second id")
	}
}
