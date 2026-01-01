package db

const sqlAddLevel = `
	INSERT INTO air.substance_level(observation_id, substance_id, level_mcg)
	VALUES ($1, $2, $3)
	ON CONFLICT (observation_id, substance_id) DO UPDATE
	SET level_mcg = excluded.level_mcg
`

func (st *Storage) addLevel(obsId int, subId int, value float32) {
	_, err := st.con.Exec(st.ctx, sqlAddLevel, obsId, subId, value)
	if err != nil {
		log.Error("unable to save level", "error", err)
	}
}
