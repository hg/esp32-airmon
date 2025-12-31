package cityair

import (
	"fmt"
	"net/url"
	"time"

	"github.com/hg/airmon/data"
	"github.com/hg/airmon/db"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/net"
	"github.com/hg/airmon/spatial"
	"go.uber.org/zap"
)

var log = logger.Get(logger.CityAir)

const base = "https://auatech-vko.kz/harvester/v2"

type schedule struct {
	nextFull time.Time
	center   spatial.Point
}

type collector struct {
	client *net.Client
	sender *db.Storage
	sched  *schedule
}

type geo struct {
	Lat float32 `json:"latitude"`
	Lon float32 `json:"longitude"`
}

type post struct {
	Id   int    `json:"id"`
	Name string `json:"name"`
	Geo  geo    `json:"geo"`
}

type meta struct {
	Units map[string]string `json:"units"`
}

func (fu *schedule) schedule() {
	fu.nextFull = time.Now().Add(3 * time.Hour)
}

func (g *geo) toPoint() spatial.Point {
	return spatial.Point{
		Lat: float64(g.Lat),
		Lon: float64(g.Lon),
	}
}

type measure struct {
	Meta meta             `json:"meta"`
	Data []map[string]any `json:"data"`
}

func parseDate(input any) (time.Time, error) {
	str, ok := input.(string)
	if !ok {
		return time.Time{}, fmt.Errorf("date is not a string: %v", input)
	}
	return time.Parse("2006-01-02T15:04:05Z", str)
}

func toFloat(raw any) (float32, bool) {
	switch val := raw.(type) {
	case float32:
		return val, true
	case float64:
		return float32(val), true
	case int:
		return float32(val), true
	case int32:
		return float32(val), true
	case int64:
		return float32(val), true
	default:
		return 0, false
	}
}

func (co *collector) loadPosts() []post {
	var posts []post

	if err := co.client.GetJSON(base+"/posts", &posts); err != nil {
		log.Error("unable to load posts", zap.Error(err))
		return nil
	}

	log.Info("loaded posts", zap.Int("count", len(posts)))
	return posts
}

func (co *collector) isPartialUpdate() bool {
	if co.sched == nil {
		return false // no coordinates, poll everything
	}
	full := co.sched.nextFull.Before(time.Now())
	if full {
		co.sched.schedule()
		log.Info("full update: also fetching data for remote stations")
	}
	return !full
}

func (co *collector) update() []data.Measure {
	var result []data.Measure

	posts := co.loadPosts()
	if len(posts) == 0 {
		return nil
	}

	var measure measure

	since := time.Now().Add(-6 * time.Hour)
	partial := co.isPartialUpdate()

	for _, post := range posts {
		if partial {
			dist := spatial.Haversine(co.sched.center, post.Geo.toPoint())
			if dist > 100_000 {
				log.Debug("skipping remote post",
					zap.Int("id", post.Id),
					zap.Float64("distance", dist))
				continue
			}
		}

		uri := fmt.Sprintf("%s/posts/%d/measurements?interval=5m&date__gt=%s",
			base, post.Id, url.QueryEscape(since.Format("2006-01-02 15:04:05Z")))

		if err := co.client.GetJSON(uri, &measure); err != nil {
			log.Error("unable to load post",
				zap.Int("postId", post.Id),
				zap.Error(err))
			continue
		}

		var obss []data.Observation

		for _, values := range measure.Data {
			dateRaw, ok := values["date"]
			if !ok {
				log.Error("date not found in measurement")
				continue
			}

			date, err := parseDate(dateRaw)
			if err != nil {
				log.Error("unable to parse date",
					zap.Any("date", dateRaw),
					zap.Error(err))
				continue
			}

			var level []data.Level

			for sub, unit := range measure.Meta.Units {
				raw, ok := values[sub]
				if !ok {
					continue
				}
				if val, ok := toFloat(raw); ok {
					level = append(level, data.Level{
						Substance: sub,
						Unit:      unit,
						Value:     val,
					})
				}
			}

			if len(level) > 0 {
				obss = append(obss, data.Observation{
					Date:  date,
					Level: level,
				})
			}
		}

		if len(obss) > 0 {
			result = append(result, data.Measure{
				Post: data.Post{
					Source: data.CityAir,
					Name:   post.Name,
					Lon:    post.Geo.Lon,
					Lat:    post.Geo.Lat,
				},
				Rows: obss,
			})
		}
	}

	return result
}

func (co *collector) collect() {
	for {
		rows := co.update()
		go co.sender.Enqueue(rows)
		log.Info("cityair updated")
		time.Sleep(5 * time.Minute)
	}
}

type Settings struct {
	Token  string
	LatLon string
}

func Start(sender *db.Storage, set Settings) {
	co := &collector{
		client: net.NewProxiedClient(),
		sender: sender,
		sched:  nil,
	}
	co.client.SetHeader("Authorization", "Bearer "+set.Token)

	if set.LatLon != "" {
		center, err := spatial.ParsePoint(set.LatLon)
		if err == nil {
			co.sched = &schedule{center: center}
			co.sched.schedule() // avoid spamming API calls on frequent restarts
		} else {
			log.Error("invalid center coordinate", zap.Error(err))
		}
	}

	go co.collect()
}
