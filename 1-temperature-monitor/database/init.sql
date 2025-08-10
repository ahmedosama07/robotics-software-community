-- Initialize temperature monitoring database

CREATE TABLE IF NOT EXISTS temperature_readings (
    id SERIAL PRIMARY KEY,
    sensor_id VARCHAR(100) NOT NULL,
    temperature DOUBLE PRECISION NOT NULL,
    humidity DOUBLE PRECISION,
    unit VARCHAR(20) NOT NULL,
    location VARCHAR(200),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for better query performance
CREATE INDEX IF NOT EXISTS idx_sensor_id ON temperature_readings(sensor_id);
CREATE INDEX IF NOT EXISTS idx_timestamp ON temperature_readings(timestamp);
CREATE INDEX IF NOT EXISTS idx_sensor_timestamp ON temperature_readings(sensor_id, timestamp);

-- Create a view for recent readings (last 24 hours)
CREATE OR REPLACE VIEW recent_readings AS
SELECT *
FROM temperature_readings
WHERE timestamp >= NOW() - INTERVAL '24 hours'
ORDER BY timestamp DESC;

-- Create a view for temperature statistics by sensor
CREATE OR REPLACE VIEW sensor_statistics AS
SELECT 
    sensor_id,
    location,
    COUNT(*) as reading_count,
    AVG(temperature) as avg_temperature,
    MIN(temperature) as min_temperature,
    MAX(temperature) as max_temperature,
    STDDEV(temperature) as temp_stddev,
    AVG(humidity) as avg_humidity,
    MIN(timestamp) as first_reading,
    MAX(timestamp) as last_reading
FROM temperature_readings
GROUP BY sensor_id, location;

-- Grant permissions to ROS user
GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO ros_user;
GRANT ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public TO ros_user;