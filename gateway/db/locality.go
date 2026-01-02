package db

import (
	"github.com/hg/airmon/data"
	"github.com/hg/airmon/spatial"
	"github.com/jackc/pgx/v5"
)

const sqlAddLocality = `
	INSERT INTO air.locality(slug, name_ru, name_kk, kato_id)
	VALUES (to_slug($1), $1, $1, 1)
	RETURNING id
`

func (st *Storage) addLocality(name string) (int, error) {
	var id int
	err := st.con.QueryRow(st.ctx, sqlAddLocality, name).Scan(&id)
	return id, err
}

const sqlGetClosest = `
	SELECT id
	FROM air.locality
	WHERE ST_DWithin(geo, ST_Point($1, $2, 4326), tolerance := 15000)
	ORDER BY geo <-> ST_Point($1, $2, 4326)
	LIMIT 1
`

func (st *Storage) getClosest(center spatial.Point) (int, error) {
	var id int
	err := st.con.QueryRow(st.ctx, sqlGetClosest, center.Lon, center.Lat).Scan(&id)
	return id, err
}

const sqlGetLocality = `
	SELECT id
	FROM air.locality
	WHERE slug = to_slug($1)
`

func (st *Storage) getLocality(name string, center spatial.Point) (int, error) {
	name = data.NormalizeLocality(name)
	var id int
	err := st.con.QueryRow(st.ctx, sqlGetLocality, name).Scan(&id)
	if err == pgx.ErrNoRows {
		id, err = st.getClosest(center)
		if err == pgx.ErrNoRows {
			id, err = st.addLocality(name)
		}
	}
	return id, err
}
