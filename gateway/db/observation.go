package db

import (
	"time"

	"github.com/hg/airmon/data"
	"github.com/jackc/pgx/v5"
)

const sqlAddObservation = `
	INSERT INTO air.observation(post_id, measured_at)
	VALUES ($1, $2)
	RETURNING id
`

func (st *Storage) addObservation(postID data.PostID, date time.Time) (int, error) {
	var id int
	err := st.con.QueryRow(st.ctx, sqlAddObservation, postID, date).Scan(&id)
	return id, err
}

const sqlGetObservation = `
	SELECT id
	FROM air.observation
	WHERE post_id = $1
		AND measured_at = $2
	LIMIT 1
`

func (st *Storage) getObservation(postID data.PostID, date time.Time) (int, error) {
	var id int
	err := st.con.QueryRow(st.ctx, sqlGetObservation, postID, date).Scan(&id)
	if err == pgx.ErrNoRows {
		id, err = st.addObservation(postID, date)
	}
	return id, err
}

const sqlGetLastAt = `
SELECT MAX(measured_at)
FROM air.observation
WHERE post_id = $1
`

func (st *Storage) GetLastAt(postID data.PostID) (time.Time, error) {
	var lastAt time.Time
	err := st.con.QueryRow(st.ctx, sqlGetLastAt, postID).Scan(&lastAt)
	if err == pgx.ErrNoRows {
		return time.Time{}, nil
	}
	return lastAt, err
}
