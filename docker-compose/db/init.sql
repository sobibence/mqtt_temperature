CREATE DATABASE db;

CREATE TABLE sensor_data (
    timestamp TIMESTAMPTZ NOT NULL,
    temperature float,
    humidity float,
    co2 float,
    sensor_id INTEGER DEFAULT 0
);

SELECT create_hypertable('sensor_data', by_range('time', INTERVAL '1 week'), if_not_exists => TRUE, migrate_data => TRUE);

ALTER TABLE sensor_data SET (
   timescaledb.enable_columnstore = true, 
   timescaledb.segmentby = 'sensor_id');

CALL add_columnstore_policy('sensor_data', INTERVAL '1 weeks');