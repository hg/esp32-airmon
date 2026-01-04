package db

import (
	"github.com/hg/airmon/data"
	"github.com/jackc/pgx/v5"
)

const sqlAddPost = `
	INSERT INTO air.monitoring_post(name_ru, name_kk, latitude, longitude,
	                                desc_ru, desc_kk, locality_id, source_id, slug)
	SELECT $1, $1, $2, $3, $4, $4, $5, source.id, to_slug($6)
	FROM air.source
	WHERE source.slug = $7
	RETURNING id
`

func (st *Storage) addPost(post data.Post) (int, error) {
	locID, err := st.getLocality(post.City, post.Geo)
	if err != nil {
		return 0, err
	}
	var id int
	err = st.con.QueryRow(st.ctx, sqlAddPost,
		post.Name,    // $1
		post.Geo.Lat, // $2
		post.Geo.Lon, // $3
		post.Address, // $4
		locID,        // $5
		post.Slug,    // $6
		post.Source,  // $7
	).Scan(&id)
	return id, err
}

const sqlGetPost = `
	SELECT post.id
	FROM air.monitoring_post post
					 JOIN air.source ON source.id = post.source_id
	WHERE post.slug = to_slug($1)
		AND source.slug = $2
	LIMIT 1
`

func (st *Storage) getPost(post data.Post) (int, error) {
	var id int
	err := st.con.QueryRow(st.ctx, sqlGetPost, post.Slug, post.Source).Scan(&id)
	if err == pgx.ErrNoRows {
		id, err = st.addPost(post)
	}
	return id, err
}
