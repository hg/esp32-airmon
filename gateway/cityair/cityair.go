package cityair

import (
	"fmt"
	"net/url"
	"strconv"
	"time"

	"github.com/hg/airmon/client"
	"github.com/hg/airmon/data"
	"github.com/hg/airmon/db"
	"github.com/hg/airmon/logger"
	"github.com/hg/airmon/spatial"
)

var log = logger.Get(logger.CityAir)

const base = "https://auatech-vko.kz/harvester/v2"

type schedule struct {
	nextFull time.Time
	center   spatial.Point
}

type collector struct {
	client *client.Client
	store  *db.Storage
	sched  *schedule
}

type geo struct {
	Lat float32 `json:"latitude"`
	Lon float32 `json:"longitude"`
}

type station struct {
	ID   int    `json:"id"`
	Name string `json:"name"`
	Geo  geo    `json:"geo"`
}

type meta struct {
	Units map[string]string `json:"units"`
}

func (sc *schedule) schedule() {
	sc.nextFull = time.Now().Add(3 * time.Hour)
}

func (g *geo) toPoint() spatial.Point {
	return spatial.Point{
		Lat: g.Lat,
		Lon: g.Lon,
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

type stations struct {
	near []station
	far  []station
}

func (co *collector) loadPosts() stations {
	var all []station

	if err := co.client.GetJSON(base+"/posts", &all); err != nil {
		log.Error("unable to load posts", "error", err)
		return stations{}
	}

	if co.sched == nil {
		log.Info("loaded posts", "all", len(all))
		return stations{near: all, far: nil}
	}

	full := co.sched.isFullUpdate()

	posts := stations{
		near: make([]station, 0, len(all)),
		far:  make([]station, 0, len(all)),
	}

	for _, post := range all {
		dist := spatial.Haversine(co.sched.center, post.Geo.toPoint())
		if dist > 100_000 {
			if full {
				posts.far = append(posts.far, post)
			} else {
				log.Debug("skipping remote post", "id", post.ID, "dist", dist)
			}
		} else {
			posts.near = append(posts.near, post)
		}
	}

	log.Info("loaded posts",
		"near", len(posts.near),
		"far", len(posts.far))

	return posts
}

func (sc *schedule) isFullUpdate() bool {
	full := sc.nextFull.Before(time.Now())
	if full {
		sc.schedule()
		log.Info("full update: also fetching data for remote stations")
	}
	return full
}

func (co *collector) update(posts []station) {
	var result []data.Measure

	for _, post := range posts {
		postID, err := co.store.GetPost(data.Post{
			Source: data.CityAir,
			Name:   post.Name,
			Slug:   strconv.Itoa(post.ID),
			Geo: spatial.Point{
				Lat: post.Geo.Lat,
				Lon: post.Geo.Lon,
			},
		})

		if err != nil {
			log.Error("unable to find post", "error", err)
			continue
		}

		since, err := co.store.GetLastAt(postID)

		if err != nil || since.IsZero() {
			if err != nil {
				log.Error("unable to get last measurement", "error", err)
			}
			since = time.Now().UTC().Add(-12 * time.Hour)
		} else {
			since = since.UTC().Add(-time.Hour)
		}

		uri := fmt.Sprintf("%s/posts/%d/measurements?interval=5m&date__gt=%s",
			base, post.ID, url.QueryEscape(since.Format("2006-01-02 15:04:05Z")))

		var ms measure

		if err := co.client.GetJSON(uri, &ms); err != nil {
			log.Error("unable to load post",
				"postId", post.ID,
				"error", err)
			continue
		}

		var obss []data.Observation

		for _, values := range ms.Data {
			dateRaw, ok := values["date"]
			if !ok {
				log.Error("date not found in measurement")
				continue
			}

			date, err := parseDate(dateRaw)
			if err != nil {
				log.Error("unable to parse date",
					"date", dateRaw,
					"error", err)
				continue
			}

			var level []data.Level

			for sub, unit := range ms.Meta.Units {
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
				PostID: postID,
				Rows:   obss,
			})
		}
	}

	go co.store.Enqueue(result)
}

func (co *collector) collect() {
	for {
		posts := co.loadPosts()
		if len(posts.near) > 0 {
			co.update(posts.near)
		}
		if len(posts.far) > 0 {
			go co.update(posts.far)
		}
		log.Info("cityair updated")
		time.Sleep(5 * time.Minute)
	}
}

type Settings struct {
	Token  string     `json:"token"`
	Center [2]float32 `json:"center"`
}

func Start(sender *db.Storage, set Settings) {
	co := &collector{
		client: client.NewProxied(),
		store:  sender,
		sched:  nil,
	}
	co.client.SetHeader("Authorization", "Bearer "+set.Token)

	center := spatial.Point{
		Lat: set.Center[0],
		Lon: set.Center[1],
	}
	if center.IsValid() {
		co.sched = &schedule{center: center}
		co.sched.schedule() // avoid spamming API calls on frequent restarts
	} else {
		log.Error("invalid center coordinate", "error", set.Center)
	}

	go co.collect()
}
