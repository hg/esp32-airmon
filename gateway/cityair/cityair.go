package cityair

import (
	"fmt"
	"net/url"
	"os"
	"strconv"
	"time"

	"github.com/hg/airmon/influx"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/net"
	influxdb2 "github.com/influxdata/influxdb-client-go/v2"
	"go.uber.org/zap"
)

var log = logger.Get(logger.CityAir)

const base = "https://auatech-vko.kz/harvester/v2"

type collector struct {
	client *net.Client
	times  map[int]time.Time
}

type geo struct {
	Offset   int     `json:"gmtOffsetSeconds"`
	Timezone string  `json:"timeZoneIana"`
	Lat      float64 `json:"latitude"`
	Lon      float64 `json:"longitude"`
}

type post struct {
	Id             int      `json:"id"`
	Name           string   `json:"name"`
	IsOnline       bool     `json:"isOnline"`
	IsPublic       bool     `json:"isPublic"`
	Geo            geo      `json:"geo"`
	DataValueTypes []string `json:"dataValueTypes"`
}

type measurement struct {
	Data []map[string]any `json:"data"`
}

type result struct {
	date   time.Time
	tags   map[string]string
	fields map[string]any
}

func parseDate(input any) (time.Time, error) {
	str, ok := input.(string)
	if !ok {
		return time.Time{}, fmt.Errorf("date is not a string: %v", input)
	}
	return time.Parse("2006-01-02T15:04:05Z", str)
}

func (col *collector) update() []*result {
	var posts []*post
	var results []*result

	if err := col.client.GetJSON(base+"/posts", &posts); err != nil {
		log.Error("unable to load posts", zap.Error(err))
		return results
	}

	log.Info("loaded posts", zap.Int("count", len(posts)))

	if len(posts) == 0 {
		return results
	}

	ms := measurement{}

	for _, ps := range posts {
		since, ok := col.times[ps.Id]
		if !ok {
			since = time.Now().Add(-24 * time.Hour)
		}

		uri := fmt.Sprintf("%s/posts/%d/measurements?interval=5m&date__gt=%s",
			base, ps.Id, url.QueryEscape(since.Format("2006-01-02 15:04:05Z")))

		if err := col.client.GetJSON(uri, &ms); err != nil {
			log.Error("unable to load post",
				zap.Int("postId", ps.Id),
				zap.Error(err))
			continue
		}

		for _, values := range ms.Data {
			createdRaw, ok := values["date"]
			if !ok {
				log.Error("date not found in measurement")
				continue
			}

			created, err := parseDate(createdRaw)
			if err != nil {
				log.Error("unable to parse date",
					zap.Any("date", createdRaw),
					zap.Error(err))
				continue
			}

			if created.After(since) {
				since = created
			}

			re := &result{
				date: created,
				tags: map[string]string{
					"city": "Усть-Каменогорск",
					"post": strconv.Itoa(ps.Id),
					"name": ps.Name,
				},
				fields: map[string]any{
					"lat":     ps.Geo.Lat,
					"lon":     ps.Geo.Lon,
					"version": values["version"],
				},
			}

			for k, v := range values {
				switch val := v.(type) {
				case float32, float64:
					re.fields[k] = val
				}
			}

			results = append(results, re)
		}

		col.times[ps.Id] = since
	}

	return results
}

func save(sender *influx.MeasurementSender, data []*result) {
	for _, row := range data {
		pt := influxdb2.NewPoint("cityair", row.tags, row.fields, row.date)
		sender.Send(pt)
	}
}

func Collect(sender *influx.MeasurementSender) {
	token := os.Getenv("CITYAIR_TOKEN")
	if token == "" {
		log.Error("cityair token not set")
		return
	}

	col := &collector{
		client: net.NewProxiedClient(),
		times:  make(map[int]time.Time),
	}
	col.client.SetHeader("Authorization", "Bearer "+token)

	for {
		rows := col.update()
		go save(sender, rows)
		time.Sleep(10 * time.Minute)
	}
}
