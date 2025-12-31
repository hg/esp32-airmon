package db

import (
	"github.com/hg/airmon/data"
	"github.com/jackc/pgx/v5"
)

const sqlAddSubstance = `
	INSERT INTO air.substance(slug, name_ru, name_kk, formula)
	VALUES (UPPER($1), $1, $1, $1)
	RETURNING id
`

func (st *Storage) addSubstance(name string) (int, error) {
	var id int
	err := st.con.QueryRow(st.ctx, sqlAddSubstance, name).Scan(&id)
	return id, err
}

const sqlGetSubstance = `
	SELECT id
	FROM air.substance
	WHERE slug = UPPER($1)
	LIMIT 1
`

func (st *Storage) getSubstance(name string) (int, error) {
	name = data.NormalizeSubstance(name)
	var id int
	err := st.con.QueryRow(st.ctx, sqlGetSubstance, name).Scan(&id)
	if err == pgx.ErrNoRows {
		id, err = st.addSubstance(name)
	}
	return id, err
}
