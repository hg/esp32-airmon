package data

import (
	"regexp"
	"strings"
)

var (
	reSpace    = regexp.MustCompile(`\s+`)
	reLocality = regexp.MustCompile(`(?i)([пс]\.)|(пос[её]ло?к|село)\s*`)
)

func NormalizeLocality(name string) string {
	name = reLocality.ReplaceAllString(name, "")
	name = strings.TrimSpace(name)
	name = reSpace.ReplaceAllString(name, " ")
	return name
}
