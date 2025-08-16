# Makefile for ROS2 Docker Temperature Monitor

.PHONY: help test test-python test-cpp test-db test-all build

help:
	@echo "Available commands:"
	@echo "  make build            - Build Docker containers"
	@echo "  make test             - Run all tests"
	@echo "  make test-python      - Run Python unit tests"
	@echo "  make test-cpp         - Run C++ unit tests"
	@echo "  make test-db          - Run database integration tests"
	@echo "  make coverage         - Run tests with coverage (Python + C++)"
	@echo "  make coverage-report  - Show coverage report URLs"
	@echo "  make clean            - Remove coverage and build artifacts"

test:
	@echo "Running all test categories..."
	@$(MAKE) build
	@$(MAKE) test-python
	@$(MAKE) test-cpp
	@$(MAKE) test-db

test-python:
	@echo "🧪 Running Python unit tests..."
	@docker exec -it temp_publisher bash -c "\
	source /opt/ros/humble/setup.bash && \
	source /workspace/install/setup.bash && \
	python3 -m pytest /workspace/src/temperature_monitor/test/ -v" || true

test-cpp:
	@echo "🧪 Running C++ unit tests..."
	@docker exec -it temp_subscriber bash -c "\
	source /opt/ros/humble/setup.bash && \
	source /workspace/install/setup.bash && \
	colcon test --packages-select temperature_monitor --ctest-args -V"
	@docker exec -it temp_subscriber bash -c "\
	source /opt/ros/humble/setup.bash && \
	source /workspace/install/setup.bash && \
	colcon test-result --all --verbose"

test-db:
	@echo "🧪 Running database integration tests..."
	@chmod +x database/test_database.sh
	@./database/test_database.sh

build:
	@echo "🏗️  Building all services with docker-compose..."
	@docker compose up --build -d
	@sleep 5  # Wait for containers to initialize

# --- COVERAGE TARGETS ---

coverage: coverage-python coverage-cpp

coverage-python:
	@echo "📊 Generating Python test coverage..."
	@mkdir -p coverage/html/python

	# Run pytest-cov inside the container
	@docker exec -it temp_publisher bash -c "\
	source /opt/ros/humble/setup.bash && \
	source /workspace/install/setup.bash && \
	cd /workspace && \
	pip3 install pytest-cov && \
	python3 -m pytest src/temperature_monitor/test/ \
	--cov=src/temperature_monitor/temperature_monitor \
	--cov-report=term \
	--cov-report=html:/workspace/coverage/html/python \
	--cov-report=xml:/workspace/coverage/python-coverage.xml \
	-v"

	# 🔁 Copy coverage reports from container to host
	@echo "📁 Copying coverage reports from container to host..."
	@docker cp temp_publisher:/workspace/coverage/html/python/. coverage/html/python/ 2>/dev/null || \
	(echo "❌ Failed to copy coverage data from container" && exit 1)

	@docker cp temp_publisher:/workspace/coverage/python-coverage.xml coverage/ 2>/dev/null || \
	echo "⚠️  No XML coverage file found (optional)"

	@echo "📄 Python coverage report is ready:"
	@echo "→ file://$(shell pwd)/coverage/html/python/index.html"

coverage-cpp:
	@echo "📊 Generating C++ test coverage..."
	@mkdir -p coverage/html/cpp

	# 🔁 Rebuild with coverage flags
	@docker exec -it temp_subscriber bash -c "\
	source /opt/ros/humble/setup.bash && \
	cd /workspace && \
	colcon build \
	--packages-select temperature_monitor \
	--cmake-args \
	-DCMAKE_BUILD_TYPE=Debug \
	-DCMAKE_CXX_FLAGS='--coverage -O0' \
	-DCMAKE_C_FLAGS='--coverage -O0'"

	# 🧪 Run tests (this generates .gcda/.gcno files in build/)
	@docker exec -it temp_subscriber bash -c "\
	source /opt/ros/humble/setup.bash && \
	source /workspace/install/setup.bash && \
	colcon test --packages-select temperature_monitor --ctest-args -V"

	# 📊 Capture coverage from the BUILD directory, not src/
	@docker exec -it temp_subscriber bash -c "\
	cd /workspace && \
	lcov --directory build --capture --output-file coverage.info && \
	genhtml coverage.info --output-directory /workspace/coverage/html/cpp"

	# 📥 Copy report to host
	@docker cp temp_subscriber:/workspace/coverage/html/cpp/. coverage/html/cpp/ 2>/dev/null || \
	echo "⚠️  Failed to copy C++ coverage to host"
	@echo "📄 C++ coverage: file://$(shell pwd)/coverage/html/cpp/index.html"

coverage-report:
	@echo "📄 Coverage reports ready!"
	@echo "→ Python: file://$(shell pwd)/coverage/html/python/index.html"
	@echo "→ C++:    file://$(shell pwd)/coverage/html/cpp/index.html"

# --- ALL TESTS WITH COVERAGE ---
all: 
	@echo "Running all tests with coverage..."
	@$(MAKE) test
	@$(MAKE) coverage

# --- CLEANUP ---

clean:
	@echo "🧹 Cleaning up coverage and build artifacts..."
	@rm -rf coverage/
	@docker exec -it temp_subscriber rm -rf /workspace/coverage /workspace/coverage.info /workspace/shared/cpp_coverage 2>/dev/null || true
	@echo "✅ Clean complete"