#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Setting up PostgreSQL database with Docker...${NC}"

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Docker is not installed. Please install Docker first.${NC}"
    exit 1
fi

# Stop any existing temperature_db container
docker stop temperature_db 2>/dev/null
docker rm temperature_db 2>/dev/null

# Start a new PostgreSQL container
echo -e "${BLUE}Starting PostgreSQL container...${NC}"
docker run --name temperature_db -e POSTGRES_USER=ros_user -e POSTGRES_PASSWORD=password -e POSTGRES_DB=temperature_db -p 5432:5432 -d postgres:13

# Wait for PostgreSQL to start
echo -e "${BLUE}Waiting for PostgreSQL to start...${NC}"
sleep 5

# Initialize the database
echo -e "${BLUE}Initializing database...${NC}"
docker cp database/init.sql temperature_db:/init.sql
docker exec temperature_db psql -U ros_user -d temperature_db -f /init.sql

# Run the tests
echo -e "${BLUE}Running database tests...${NC}"

# Function to run SQL commands
run_psql() {
    docker exec temperature_db psql -U ros_user -d temperature_db -c "$1"
}

# Function to check command success
check_success() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ $1${NC}"
        return 0
    else
        echo -e "${RED}✗ $1${NC}"
        return 1
    fi
}

# Test 1: Check if database is accessible
echo "1. Testing database connection..."
run_psql "\dt"
check_success "Database connection successful" || exit 1

# Test 2: Check if tables exist
echo "2. Checking table existence..."
run_psql "\dt" | grep -q "temperature_readings"
check_success "temperature_readings table exists"

# Test 3: Check table structure
echo "3. Verifying table structure..."
run_psql "\d temperature_readings" | grep -q "sensor_id"
check_success "Table has sensor_id column"

run_psql "\d temperature_readings" | grep -q "temperature"
check_success "Table has temperature column"

run_psql "\d temperature_readings" | grep -q "timestamp"
check_success "Table has timestamp column"

# Test 4: Check indexes
echo "4. Verifying indexes..."
run_psql "\di" | grep -q "idx_sensor_id"
check_success "idx_sensor_id index exists"

run_psql "\di" | grep -q "idx_timestamp"
check_success "idx_timestamp index exists"

# Test 5: Check views
echo "5. Verifying views..."
run_psql "\dv" | grep -q "recent_readings"
check_success "recent_readings view exists"

run_psql "\dv" | grep -q "sensor_statistics"
check_success "sensor_statistics view exists"

# Test 6: Test data insertion
echo "6. Testing data insertion..."
run_psql "INSERT INTO temperature_readings (sensor_id, temperature, humidity, unit, location) 
          VALUES ('test_sensor', 25.5, 45.0, 'Celsius', 'Test Location')"
check_success "Data insertion successful"

# Test 7: Test data retrieval
echo "7. Testing data retrieval..."
run_psql "SELECT * FROM temperature_readings WHERE sensor_id = 'test_sensor'" | grep -q "25.5"
check_success "Data retrieval successful"

# Test 8: Test views functionality
echo "8. Testing views..."
run_psql "SELECT * FROM recent_readings" | grep -q "test_sensor"
check_success "Recent readings view works"

run_psql "SELECT * FROM sensor_statistics" | grep -q "test_sensor"
check_success "Sensor statistics view works"

# Test 9: Clean up test data
echo "9. Cleaning up test data..."
run_psql "DELETE FROM temperature_readings WHERE sensor_id = 'test_sensor'"
check_success "Test data cleaned up"

echo -e "${GREEN}All database tests passed successfully!${NC}"

# Stop and remove the container
echo -e "${BLUE}Cleaning up Docker container...${NC}"
docker stop temperature_db >/dev/null
docker rm temperature_db >/dev/null

echo -e "${GREEN}Database testing completed!${NC}"