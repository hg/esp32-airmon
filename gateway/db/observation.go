package db

import (
	"time"

	"github.com/jackc/pgx/v5"
)

const sqlAddObservation = `
	INSERT INTO air.observation(post_id, measured_at)
	VALUES ($1, $2)
	RETURNING id
`

func (st *Storage) addObservation(postId int, date time.Time) (int, error) {
	var id int
	err := st.con.QueryRow(st.ctx, sqlAddObservation, postId, date).Scan(&id)
	return id, err
}

const sqlGetObservation = `
	SELECT id
	FROM air.observation
	WHERE post_id = $1
		AND measured_at = $2
	LIMIT 1
`

func (st *Storage) getObservation(postId int, date time.Time) (int, error) {
	var id int
	err := st.con.QueryRow(st.ctx, sqlGetObservation, postId, date).Scan(&id)
	if err == pgx.ErrNoRows {
		id, err = st.addObservation(postId, date)
	}
	return id, err
}
