#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import asyncio
import websockets
import json
import threading
from datetime import datetime, timedelta
from aiohttp import web
import os
import signal
import sys
import logging
import time
from decimal import Decimal

# Database imports
import psycopg2
from psycopg2.extras import RealDictCursor

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Try to import the temperature message, with fallback
try:
    from temperature_interfaces.msg import TemperatureReading
    logger.info("Successfully imported temperature_interfaces.msg.TemperatureReading")
except ImportError as e:
    logger.error(f"Failed to import temperature_interfaces: {e}")
    # Create a dummy message class for testing
    class TemperatureReading:
        def __init__(self):
            self.temperature = 0.0
            self.humidity = 0.0
            self.sensor_id = ""
            self.unit = ""
            self.location = ""
            self.timestamp = None
    logger.warning("Using dummy TemperatureReading class")


# Check if files exist and provide helpful error messages
def check_files_exist():
    """Check if required files exist and provide helpful error messages if not."""
    base_dir = os.path.dirname(__file__)
    templates_dir = os.path.join(base_dir, 'templates')
    static_dir = os.path.join(base_dir, 'static')

    if not os.path.exists(templates_dir):
        logger.error(f"Templates directory not found: {templates_dir}")
    if not os.path.exists(static_dir):
        logger.error(f"Static directory not found: {static_dir}")

    required_files = {
        os.path.join(templates_dir, 'index.html'): 'Main HTML template',
        os.path.join(templates_dir, 'debug.html'): 'Debug HTML template',
        os.path.join(static_dir, 'css', 'style.css'): 'CSS stylesheet',
        os.path.join(static_dir, 'js', 'script.js'): 'JavaScript file',
        os.path.join(static_dir, 'lib', 'chart.js'): 'Library script file'
    }

    missing_files = []
    for file_path, description in required_files.items():
        if not os.path.exists(file_path):
            missing_files.append(f"{description}: {file_path}")

    if missing_files:
        logger.error("Missing required files:")
        for missing_file in missing_files:
            logger.error(f"  - {missing_file}")
        logger.error("Current working directory: %s", os.getcwd())
        logger.error("Contents of /workspace/src: %s", os.listdir('/workspace/src') if os.path.exists('/workspace/src') else "Directory not found")
        return False

    return True

class DatabaseManager:
    """Enhanced database manager with additional query methods."""
    
    def __init__(self, db_config=None):
        if db_config is None:
            self.db_config = {
                'host': 'temperature-db',  # Docker container name
                'port': 5432,
                'database': 'temperature_data',
                'user': 'ros_user',
                'password': 'ros_password'
            }
        else:
            self.db_config = db_config
            
        self.connection = None
        self.lock = threading.Lock()
        self.connected = False
        
        # Connect to database with retry logic
        self._connect_with_retry()
    
    def _connect_with_retry(self):
        """Connect to database with retry logic."""
        max_retries = 10
        retry_delay = 2
        
        for attempt in range(max_retries):
            try:
                logger.info(f"Database connection attempt {attempt + 1}/{max_retries}...")
                self.connection = psycopg2.connect(
                    **self.db_config,
                    cursor_factory=RealDictCursor
                )
                self.connection.autocommit = True
                self.connected = True
                logger.info("✅ Database connection successful!")
                return True
                
            except psycopg2.Error as e:
                logger.warning(f"Database connection attempt {attempt + 1} failed: {e}")
                if attempt < max_retries - 1:
                    logger.info(f"Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                    retry_delay = min(retry_delay * 1.5, 30)  # Max 30 second delay
                else:
                    logger.error("❌ All database connection attempts failed")
                    self.connected = False
                    return False
    
    def is_connected(self):
        """Check if database is connected."""
        try:
            return self.connected and self.connection and not self.connection.closed
        except:
            return False
    
    def ensure_connection(self):
        """Ensure database connection is active."""
        if not self.is_connected():
            logger.warning("Database connection lost, attempting to reconnect...")
            return self._connect_with_retry()
        return True
    
    def store_temperature_reading(self, temperature_data):
        """Store temperature reading in database."""
        if not self.ensure_connection():
            logger.error("Cannot store temperature reading - database not connected")
            return False
            
        try:
            with self.lock:
                cursor = self.connection.cursor()
                
                insert_query = """
                    INSERT INTO temperature_readings 
                    (sensor_id, temperature, humidity, unit, location, timestamp)
                    VALUES (%(sensor_id)s, %(temperature)s, %(humidity)s, %(unit)s, %(location)s, NOW())
                """
                
                cursor.execute(insert_query, temperature_data)
                cursor.close()
                
                logger.debug(f"✅ Stored: {temperature_data['sensor_id']} - {temperature_data['temperature']:.2f}°C")
                return True
                
        except psycopg2.Error as e:
            logger.error(f"❌ Database error: {e}")
            self.connected = False  # Trigger reconnect
            return False
        except Exception as e:
            logger.error(f"❌ Unexpected error storing data: {e}")
            return False
    
    def get_recent_readings(self, limit=50, hours=24):
        """Get recent temperature readings within specified hours."""
        if not self.ensure_connection():
            logger.warning("Database not connected for recent readings")
            return []
            
        try:
            with self.lock:
                cursor = self.connection.cursor()
                cursor.execute("""
                    SELECT sensor_id, temperature, humidity, unit, location, timestamp
                    FROM temperature_readings 
                    WHERE timestamp >= NOW() - INTERVAL '%s hours'
                    ORDER BY timestamp DESC 
                    LIMIT %s
                """, (hours, limit))
                results = cursor.fetchall()
                cursor.close()
                
                # Convert to list of dicts with ISO timestamp
                readings = []
                for row in results:
                    reading = dict(row)
                    reading = self._convert_decimals_to_floats(reading)  # Convert Decimals to floats
                    reading['timestamp'] = reading['timestamp'].isoformat()
                    readings.append(reading)
                
                logger.info(f"Retrieved {len(readings)} recent readings")
                return readings
                
        except psycopg2.Error as e:
            logger.error(f"Error retrieving recent readings: {e}")
            self.connected = False
            return []
        except Exception as e:
            logger.error(f"Unexpected error retrieving recent readings: {e}")
            return []
    
    def get_sensor_statistics(self):
        """Get statistics for all sensors."""
        if not self.ensure_connection():
            logger.warning("Database not connected for statistics")
            return []
            
        try:
            with self.lock:
                cursor = self.connection.cursor()
                cursor.execute("""
                    SELECT 
                        sensor_id,
                        location,
                        COUNT(*) as reading_count,
                        AVG(temperature)::NUMERIC(10,2) as avg_temperature,
                        MIN(temperature)::NUMERIC(10,2) as min_temperature,
                        MAX(temperature)::NUMERIC(10,2) as max_temperature,
                        STDDEV(temperature)::NUMERIC(10,2) as temp_stddev,
                        AVG(humidity)::NUMERIC(10,2) as avg_humidity,
                        MIN(timestamp) as first_reading,
                        MAX(timestamp) as last_reading
                    FROM temperature_readings
                    WHERE timestamp >= NOW() - INTERVAL '24 hours'
                    GROUP BY sensor_id, location
                    ORDER BY sensor_id
                """)
                results = cursor.fetchall()
                cursor.close()
                
                # Convert timestamps to ISO format
                stats = []
                for row in results:
                    stat = dict(row)
                    stat = self._convert_decimals_to_floats(stat)  # Convert Decimals to floats
                    stat['first_reading'] = stat['first_reading'].isoformat()
                    stat['last_reading'] = stat['last_reading'].isoformat()
                    stats.append(stat)
                
                logger.info(f"Retrieved statistics for {len(stats)} sensors")
                return stats
                
        except psycopg2.Error as e:
            logger.error(f"Error retrieving sensor statistics: {e}")
            self.connected = False
            return []
        except Exception as e:
            logger.error(f"Unexpected error retrieving statistics: {e}")
            return []
    
    def get_hourly_averages(self, sensor_id=None, hours=24):
        """Get hourly temperature averages."""
        if not self.ensure_connection():
            logger.warning("Database not connected for hourly averages")
            return []
            
        try:
            with self.lock:
                cursor = self.connection.cursor()
                
                if sensor_id:
                    cursor.execute("""
                        SELECT 
                            DATE_TRUNC('hour', timestamp) as hour,
                            sensor_id,
                            AVG(temperature)::NUMERIC(10,2) as avg_temp,
                            AVG(humidity)::NUMERIC(10,2) as avg_humidity,
                            COUNT(*) as reading_count
                        FROM temperature_readings 
                        WHERE sensor_id = %s 
                        AND timestamp >= NOW() - INTERVAL %s
                        GROUP BY DATE_TRUNC('hour', timestamp), sensor_id
                        ORDER BY hour DESC
                    """, (sensor_id, f"{hours} hours"))
                else:
                    cursor.execute("""
                        SELECT 
                            DATE_TRUNC('hour', timestamp) as hour,
                            sensor_id,
                            AVG(temperature)::NUMERIC(10,2) as avg_temp,
                            AVG(humidity)::NUMERIC(10,2) as avg_humidity,
                            COUNT(*) as reading_count
                        FROM temperature_readings 
                        WHERE timestamp >= NOW() - INTERVAL %s
                        GROUP BY DATE_TRUNC('hour', timestamp), sensor_id
                        ORDER BY hour DESC, sensor_id
                    """, (f"{hours} hours",))
                
                results = cursor.fetchall()
                cursor.close()
                
                # Convert timestamps to ISO format
                averages = []
                for row in results:
                    avg = dict(row)
                    avg = self._convert_decimals_to_floats(avg)  # Convert Decimals to floats
                    avg['hour'] = avg['hour'].isoformat()
                    averages.append(avg)
                
                logger.info(f"Retrieved {len(averages)} hourly averages")
                return averages
                
        except psycopg2.Error as e:
            logger.error(f"Error retrieving hourly averages: {e}")
            self.connected = False
            return []
        except Exception as e:
            logger.error(f"Unexpected error retrieving hourly averages: {e}")
            return []
    
    def get_temperature_trends(self, sensor_id=None, hours=24):
        """Get temperature trends and analysis."""
        if not self.ensure_connection():
            logger.warning("Database not connected for trends")
            return {}
            
        try:
            with self.lock:
                cursor = self.connection.cursor()
                
                where_clause = "WHERE timestamp >= NOW() - INTERVAL %s"
                params = [f"{hours} hours"]
                
                if sensor_id:
                    where_clause += " AND sensor_id = %s"
                    params.append(sensor_id)
                
                cursor.execute(f"""
                    WITH trend_data AS (
                        SELECT 
                            temperature,
                            timestamp,
                            LAG(temperature) OVER (ORDER BY timestamp) as prev_temp,
                            LAG(timestamp) OVER (ORDER BY timestamp) as prev_timestamp
                        FROM temperature_readings 
                        {where_clause}
                        ORDER BY timestamp
                    )
                    SELECT 
                        COUNT(*) as total_readings,
                        AVG(temperature)::NUMERIC(10,2) as avg_temp,
                        MIN(temperature)::NUMERIC(10,2) as min_temp,
                        MAX(temperature)::NUMERIC(10,2) as max_temp,
                        STDDEV(temperature)::NUMERIC(10,2) as temp_stddev,
                        COUNT(CASE WHEN temperature > prev_temp THEN 1 END) as rising_count,
                        COUNT(CASE WHEN temperature < prev_temp THEN 1 END) as falling_count,
                        COUNT(CASE WHEN ABS(temperature - prev_temp) <= 0.5 THEN 1 END) as stable_count
                    FROM trend_data
                    WHERE prev_temp IS NOT NULL
                """, params)
                
                result = cursor.fetchone()
                cursor.close()
                
                if result:
                    trend = dict(result)
                    trend = self._convert_decimals_to_floats(trend)  # Convert Decimals to floats
                    # Calculate trend percentages
                    total = trend['rising_count'] + trend['falling_count'] + trend['stable_count']
                    if total > 0:
                        trend['rising_percentage'] = round((trend['rising_count'] / total) * 100, 1)
                        trend['falling_percentage'] = round((trend['falling_count'] / total) * 100, 1)
                        trend['stable_percentage'] = round((trend['stable_count'] / total) * 100, 1)
                    else:
                        trend['rising_percentage'] = 0
                        trend['falling_percentage'] = 0
                        trend['stable_percentage'] = 0
                    
                    logger.info(f"Retrieved trend data: {trend.get('total_readings', 0)} readings analyzed")
                    return trend
                
                logger.warning("No trend data available")
                return {}
                
        except psycopg2.Error as e:
            logger.error(f"Error retrieving temperature trends: {e}")
            self.connected = False
            return {}
        except Exception as e:
            logger.error(f"Unexpected error retrieving trends: {e}")
            return {}
    
    def disconnect(self):
        """Close database connection."""
        with self.lock:
            if self.connection:
                self.connection.close()
                self.connected = False
                logger.info("Database connection closed")

    def _convert_decimals_to_floats(self, obj):
        """Recursively convert Decimal objects to float for JSON serialization."""
        if isinstance(obj, Decimal):
            return float(obj)
        elif isinstance(obj, dict):
            return {k: self._convert_decimals_to_floats(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [self._convert_decimals_to_floats(item) for item in obj]
        else:
            return obj


class WebVisualizer(Node):
    """
    Enhanced ROS2 Node that provides web-based visualization with database integration.
    """
    
    def __init__(self):
        super().__init__('web_visualizer')
        
        # Data storage
        self.latest_data = None
        self.connected_clients = set()
        self.should_stop = False
        
        # Initialize database manager
        self.db_manager = DatabaseManager()
        
        # Set up QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscription
        self.subscription = self.create_subscription(
            TemperatureReading,
            'temperature_data',
            self.temperature_callback,
            qos_profile
        )
        
        self.get_logger().info('Enhanced Web Visualizer initialized with database integration')
        self.get_logger().info(f'Database connected: {self.db_manager.is_connected()}')
        
        # Set up graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signals gracefully."""
        self.get_logger().info(f'Received signal {sig}, shutting down...')
        self.should_stop = True
        if self.db_manager:
            self.db_manager.disconnect()
    
    def temperature_callback(self, msg):
        """Process incoming temperature data and broadcast to web clients."""
        try:
            # Convert ROS message to dict for JSON serialization
            self.latest_data = {
                'type': 'live_data',
                'temperature': msg.temperature,
                'humidity': msg.humidity,
                'sensor_id': msg.sensor_id,
                'unit': msg.unit,
                'location': msg.location,
                'timestamp': datetime.now().isoformat()
            }
            
            # Store data in database
            db_stored = False
            if self.db_manager and self.db_manager.is_connected():
                db_stored = self.db_manager.store_temperature_reading(self.latest_data)
            
            # Log with database status
            db_status = "✓" if db_stored else "✗"
            self.get_logger().info(f'[DB:{db_status}] Received: {msg.temperature:.2f}°C from {msg.sensor_id}')
            
            # If database storage failed, try to reconnect for next time
            if not db_stored and self.db_manager:
                self.get_logger().warning("Database storage failed, will retry on next message")
                
        except Exception as e:
            self.get_logger().error(f'Error processing temperature data: {str(e)}')


class WebServer:
    """Enhanced HTTP and WebSocket server with database API endpoints."""
    
    def __init__(self, visualizer_node):
        self.visualizer = visualizer_node
        self.websocket_server = None

        # Load static files
        self.static_dir = os.path.join(os.path.dirname(__file__), 'static')
        self.templates_dir = os.path.join(os.path.dirname(__file__), 'templates')

        # Check if files exist
        if not check_files_exist():
            logger.warning("Some required files are missing. The web interface may not work properly.")

    async def serve_html(self, request):
        """Serve the main HTML page."""
        try:
            with open(os.path.join(self.templates_dir, 'index.html'), 'r') as f:
                content = f.read()
            return web.Response(text=content, content_type='text/html')
        except Exception as e:
            logger.error(f"Error serving HTML: {e}")
            return web.Response(text="Error loading page", status=500)

    async def serve_css(self, request):
        """Serve CSS files."""
        try:
            with open(os.path.join(self.static_dir, 'css', 'style.css'), 'r') as f:
                content = f.read()
            return web.Response(text=content, content_type='text/css')
        except Exception as e:
            logger.error(f"Error serving CSS: {e}")
            return web.Response(text="", status=404)

    async def serve_js(self, request):
        """Serve JavaScript files."""
        try:
            with open(os.path.join(self.static_dir, 'js', 'script.js'), 'r') as f:
                content = f.read()
            return web.Response(text=content, content_type='application/javascript')
        except Exception as e:
            logger.error(f"Error serving JS: {e}")
            return web.Response(text="", status=404)
        
    async def serve_lib_chart(self, request):
        """Serve Chart.js from lib/ directory."""
        try:
            with open(os.path.join(self.static_dir, 'lib', 'chart.js'), 'r') as f:
                content = f.read()
            return web.Response(text=content, content_type='application/javascript')
        except Exception as e:
            logger.error(f"Error serving chart.js: {e}")
            return web.Response(text="File not found", status=404)

    async def serve_debug_html(self, request):
        """Serve the debug HTML page."""
        try:
            with open(os.path.join(self.templates_dir, 'debug.html'), 'r') as f:
                content = f.read()
            response = web.Response(text=content, content_type='text/html')
            response.headers['Access-Control-Allow-Origin'] = '*'  # Or restrict to your domain
            response.headers['Access-Control-Allow-Methods'] = 'GET, POST'
            response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
            return response
        except Exception as e:
            logger.error(f"Error serving debug HTML: {e}")
            return web.Response(text="Error loading debug page", status=500)

    
    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections."""
        logger.info(f'New WebSocket connection from {websocket.remote_address}')
        
        try:
            self.visualizer.connected_clients.add(websocket)
            logger.info(f'WebSocket client connected. Total: {len(self.visualizer.connected_clients)}')
            
            # Send welcome message
            welcome_msg = {
                'type': 'welcome',
                'message': 'Connected to Enhanced ROS2 Temperature Monitor',
                'timestamp': datetime.now().isoformat(),
                'database_connected': self.visualizer.db_manager.is_connected() if self.visualizer.db_manager else False,
                'features': ['live_data', 'historical_data', 'statistics', 'trends']
            }
            await websocket.send(json.dumps(welcome_msg))
            
            # Send latest data if available
            if self.visualizer.latest_data:
                await websocket.send(json.dumps(self.visualizer.latest_data))
            
            # Keep connection alive
            async for message in websocket:
                logger.debug(f'WebSocket message: {message}')
                try:
                    data = json.loads(message)
                    response = await self.handle_websocket_request(data)
                    if response:
                        await websocket.send(json.dumps(response))
                except json.JSONDecodeError:
                    await websocket.send(f"Echo: {message}")
                except Exception as e:
                    logger.error(f'Error handling WebSocket message: {e}')
                
        except websockets.exceptions.ConnectionClosed:
            logger.info('WebSocket client disconnected')
        except Exception as e:
            logger.error(f'WebSocket error: {str(e)}')
        finally:
            self.visualizer.connected_clients.discard(websocket)
            logger.info(f'WebSocket client removed. Total: {len(self.visualizer.connected_clients)}')
    
    async def handle_websocket_request(self, data):
        """Handle different types of WebSocket requests."""
        request_type = data.get('type')
        logger.info(f'Handling WebSocket request: {request_type}')
        
        if request_type == 'heartbeat':
            return {
                'type': 'heartbeat_response',
                'timestamp': datetime.now().isoformat(),
                'database_connected': self.visualizer.db_manager.is_connected()
            }
        
        elif request_type == 'get_recent_data':
            limit = data.get('limit', 50)
            hours = data.get('hours', 24)
            logger.info(f'Fetching recent data: limit={limit}, hours={hours}')
            recent_data = self.visualizer.db_manager.get_recent_readings(limit, hours)
            return {
                'type': 'recent_data',
                'data': recent_data,
                'count': len(recent_data),
                'timestamp': datetime.now().isoformat()
            }
        
        elif request_type == 'get_statistics':
            logger.info('Fetching sensor statistics')
            stats = self.visualizer.db_manager.get_sensor_statistics()
            return {
                'type': 'statistics',
                'data': stats,
                'count': len(stats),
                'timestamp': datetime.now().isoformat()
            }
        
        elif request_type == 'get_hourly_averages':
            sensor_id = data.get('sensor_id')
            hours = data.get('hours', 24)
            logger.info(f'Fetching hourly averages: sensor_id={sensor_id}, hours={hours}')
            averages = self.visualizer.db_manager.get_hourly_averages(sensor_id, hours)
            return {
                'type': 'hourly_averages',
                'data': averages,
                'count': len(averages),
                'timestamp': datetime.now().isoformat()
            }
        
        elif request_type == 'get_trends':
            sensor_id = data.get('sensor_id')
            hours = data.get('hours', 24)
            logger.info(f'Fetching trends: sensor_id={sensor_id}, hours={hours}')
            trends = self.visualizer.db_manager.get_temperature_trends(sensor_id, hours)
            return {
                'type': 'trends',
                'data': trends,
                'timestamp': datetime.now().isoformat()
            }
        
        logger.warning(f'Unknown WebSocket request type: {request_type}')
        return None
    
    # HTTP API endpoints
    async def api_recent_readings(self, request):
        """API endpoint for recent readings."""
        try:
            limit = int(request.query.get('limit', 50))
            hours = int(request.query.get('hours', 24))
            
            readings = self.visualizer.db_manager.get_recent_readings(limit, hours)
            
            return web.json_response({
                'success': True,
                'data': readings,
                'count': len(readings),
                'timestamp': datetime.now().isoformat()
            })
        except Exception as e:
            return web.json_response({
                'success': False,
                'error': str(e),
                'timestamp': datetime.now().isoformat()
            }, status=500)
    
    async def api_statistics(self, request):
        """API endpoint for sensor statistics."""
        try:
            stats = self.visualizer.db_manager.get_sensor_statistics()
            
            return web.json_response({
                'success': True,
                'data': stats,
                'count': len(stats),
                'timestamp': datetime.now().isoformat()
            })
        except Exception as e:
            return web.json_response({
                'success': False,
                'error': str(e),
                'timestamp': datetime.now().isoformat()
            }, status=500)
    
    async def api_hourly_averages(self, request):
        """API endpoint for hourly averages."""
        try:
            sensor_id = request.query.get('sensor_id')
            hours = int(request.query.get('hours', 24))
            
            averages = self.visualizer.db_manager.get_hourly_averages(sensor_id, hours)
            
            return web.json_response({
                'success': True,
                'data': averages,
                'count': len(averages),
                'timestamp': datetime.now().isoformat()
            })
        except Exception as e:
            return web.json_response({
                'success': False,
                'error': str(e),
                'timestamp': datetime.now().isoformat()
            }, status=500)
    
    async def api_trends(self, request):
        """API endpoint for temperature trends."""
        try:
            sensor_id = request.query.get('sensor_id')
            hours = int(request.query.get('hours', 24))
            
            trends = self.visualizer.db_manager.get_temperature_trends(sensor_id, hours)
            
            return web.json_response({
                'success': True,
                'data': trends,
                'timestamp': datetime.now().isoformat()
            })
        except Exception as e:
            return web.json_response({
                'success': False,
                'error': str(e),
                'timestamp': datetime.now().isoformat()
            }, status=500)
    async def health_check(self, request):
        return web.json_response({
            'status': 'healthy',
            'service': 'ros2_temp_visualizer',
            'timestamp': datetime.now().isoformat(),
            'database_connected': self.visualizer.db_manager.is_connected() if self.visualizer.db_manager else False
        })

async def broadcast_loop(visualizer):
    """Continuous loop to broadcast live data to WebSocket clients."""
    while not visualizer.should_stop:
        if visualizer.latest_data and visualizer.connected_clients:
            message = json.dumps(visualizer.latest_data)
            disconnected = set()
            
            for websocket in visualizer.connected_clients:
                try:
                    await websocket.send(message)
                except Exception as e:
                    logger.error(f"Error broadcasting to client: {e}")
                    disconnected.add(websocket)
            
            visualizer.connected_clients -= disconnected
            
        await asyncio.sleep(1.0)

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    # Create ROS2 node
    visualizer = WebVisualizer()
    web_server = WebServer(visualizer)
    
    # Set up web application with API routes
    app = web.Application()
    app.router.add_get('/', web_server.serve_html)
    app.router.add_get('/debug', web_server.serve_debug_html)
    app.router.add_get('/static/css/style.css', web_server.serve_css)
    app.router.add_get('/static/js/script.js', web_server.serve_js)
    app.router.add_get('/static/lib/chart.js', web_server.serve_lib_chart)
    
    # API endpoints
    app.router.add_get('/api/recent', web_server.api_recent_readings)
    app.router.add_get('/api/statistics', web_server.api_statistics)
    app.router.add_get('/api/hourly', web_server.api_hourly_averages)
    app.router.add_get('/api/trends', web_server.api_trends)
    app.router.add_get('/api/health', web_server.health_check)
    async def run_servers():
        """Run both HTTP and WebSocket servers."""
        try:
            # Start WebSocket server on port 8081
            logger.info("Starting WebSocket server on 0.0.0.0:8081")
            websocket_server = await websockets.serve(
                web_server.websocket_handler, 
                "0.0.0.0", 
                8081,
                ping_interval=20,
                ping_timeout=10
            )
            
            # Start HTTP server on port 8080
            logger.info("Starting HTTP server on 0.0.0.0:8080")
            runner = web.AppRunner(app)
            await runner.setup()
            site = web.TCPSite(runner, '0.0.0.0', 8080)
            await site.start()
            
            visualizer.get_logger().info('Enhanced web server started on http://0.0.0.0:8080')
            visualizer.get_logger().info('WebSocket server started on ws://0.0.0.0:8081')
            visualizer.get_logger().info('API endpoints available: /api/recent, /api/statistics, /api/hourly, /api/trends')
            
            # Start broadcast loop
            broadcast_task = asyncio.create_task(broadcast_loop(visualizer))
            
            # Wait for shutdown signal
            while not visualizer.should_stop:
                await asyncio.sleep(1)
                
            # Cleanup
            broadcast_task.cancel()
            websocket_server.close()
            await websocket_server.wait_closed()
            await runner.cleanup()
            
        except Exception as e:
            visualizer.get_logger().error(f'Server error: {str(e)}')
            logger.error(f'Server error: {str(e)}')
    
    def run_ros():
        """Run ROS2 node in separate thread."""
        try:
            rclpy.spin(visualizer)
        except Exception as e:
            visualizer.get_logger().error(f'ROS2 error: {str(e)}')
    
    # Start ROS2 in a separate thread
    ros_thread = threading.Thread(target=run_ros, daemon=True)
    ros_thread.start()
    
    # Run web servers
    try:
        asyncio.run(run_servers())
    except KeyboardInterrupt:
        visualizer.get_logger().info('Keyboard interrupt received')
    except Exception as e:
        visualizer.get_logger().error(f'Main error: {str(e)}')
        logger.error(f'Main error: {str(e)}')
    finally:
        visualizer.should_stop = True
        if rclpy.ok():
            visualizer.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()