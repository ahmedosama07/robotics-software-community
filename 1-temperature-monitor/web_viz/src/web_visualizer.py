#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import asyncio
import websockets
import json
import threading
from datetime import datetime
from aiohttp import web
import os
import signal
import sys
import logging

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

class WebVisualizer(Node):
    """
    ROS2 Node that provides a web-based visualization of temperature data.
    """
    
    def __init__(self):
        super().__init__('web_visualizer')
        
        # Data storage
        self.latest_data = None
        self.connected_clients = set()
        self.should_stop = False
        
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
        
        self.get_logger().info('Web Visualizer initialized')
        
        # Set up graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signals gracefully."""
        self.get_logger().info(f'Received signal {sig}, shutting down...')
        self.should_stop = True
    
    def temperature_callback(self, msg):
        """Process incoming temperature data and broadcast to web clients."""
        try:
            # Convert ROS message to dict for JSON serialization
            self.latest_data = {
                'temperature': msg.temperature,
                'humidity': msg.humidity,
                'sensor_id': msg.sensor_id,
                'unit': msg.unit,
                'location': msg.location,
                'timestamp': datetime.now().isoformat()
            }
            
            # Store data in database if applicable
            if hasattr(self, 'db_manager') and self.db_manager.is_connected():
                self.db_manager.store_temperature_reading(self.latest_data)

            # Log data received
            self.get_logger().info(f'Received data: {msg.temperature:.2f}°C from {msg.sensor_id}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing temperature data: {str(e)}')

class WebServer:
    """HTTP and WebSocket server for the temperature visualization."""
    
    def __init__(self, visualizer_node):
        self.visualizer = visualizer_node
        self.websocket_server = None
    
    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections."""
        logger.info(f'New WebSocket connection attempt from {websocket.remote_address}')
        
        try:
            self.visualizer.connected_clients.add(websocket)
            logger.info(f'WebSocket client connected successfully. Total: {len(self.visualizer.connected_clients)}')
            
            # Send a welcome message
            welcome_msg = {
                'type': 'welcome',
                'message': 'Connected to ROS2 Temperature Monitor',
                'timestamp': datetime.now().isoformat()
            }
            await websocket.send(json.dumps(welcome_msg))
            
            # Send latest data immediately if available
            if self.visualizer.latest_data:
                await websocket.send(json.dumps(self.visualizer.latest_data))
            
            # Keep connection alive
            async for message in websocket:
                logger.debug(f'Received WebSocket message: {message}')
                # Handle incoming messages (like heartbeat)
                try:
                    data = json.loads(message)
                    if data.get('type') == 'heartbeat':
                        response = {'type': 'heartbeat_response', 'timestamp': datetime.now().isoformat()}
                        await websocket.send(json.dumps(response))
                except json.JSONDecodeError:
                    # Echo back non-JSON messages
                    await websocket.send(f"Echo: {message}")
                except Exception as e:
                    logger.error(f'Error handling WebSocket message: {e}')
                
        except websockets.exceptions.ConnectionClosed:
            logger.info('WebSocket client disconnected normally')
        except Exception as e:
            logger.error(f'WebSocket error: {str(e)}', exc_info=True)
        finally:
            self.visualizer.connected_clients.discard(websocket)
            logger.info(f'WebSocket client disconnected. Total: {len(self.visualizer.connected_clients)}')
    
    async def serve_html(self, request):
        """Serve the HTML visualization page."""
        html_content = """
<!DOCTYPE html>
<html>
<head>
    <title>Temperature Monitor</title>
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 20px; 
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            color: white;
        }
        .container { 
            max-width: 1200px; 
            margin: 0 auto; 
        }
        .card { 
            background: rgba(255,255,255,0.1); 
            backdrop-filter: blur(10px);
            border-radius: 15px; 
            padding: 25px; 
            margin: 20px 0; 
            box-shadow: 0 8px 32px rgba(31, 38, 135, 0.37);
            border: 1px solid rgba(255, 255, 255, 0.18);
        }
        .temp-display { 
            font-size: 4em; 
            color: #fff; 
            text-align: center; 
            font-weight: bold;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .humidity-display { 
            font-size: 2.5em; 
            color: #e0e0e0; 
            text-align: center; 
            margin-top: 10px;
        }
        .info { 
            display: grid; 
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); 
            gap: 20px; 
            margin-top: 20px;
        }
        .info-item { 
            text-align: center; 
            background: rgba(255,255,255,0.05);
            padding: 15px;
            border-radius: 10px;
        }
        .info-label { 
            font-weight: bold; 
            color: #ccc; 
            font-size: 0.9em;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .info-value { 
            font-size: 1.4em; 
            color: #fff; 
            margin-top: 5px;
            font-weight: 500;
        }
        .status { 
            text-align: center; 
            padding: 15px; 
            border-radius: 10px; 
            margin: 10px 0; 
            font-weight: bold;
        }
        .connected { 
            background: rgba(76, 175, 80, 0.3); 
            color: #4caf50; 
            border: 1px solid rgba(76, 175, 80, 0.5);
        }
        .disconnected { 
            background: rgba(244, 67, 54, 0.3); 
            color: #f44336; 
            border: 1px solid rgba(244, 67, 54, 0.5);
        }
        .chart { 
            height: 400px; 
            margin: 20px 0; 
            background: rgba(255,255,255,0.05);
            border-radius: 10px;
            padding: 20px;
        }
        h1 {
            text-align: center;
            font-size: 2.5em;
            margin-bottom: 30px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        h2 {
            color: #fff;
            margin-bottom: 20px;
            font-size: 1.5em;
        }
        .debug-info {
            font-size: 0.8em;
            color: #ccc;
            margin-top: 10px;
        }
    </style>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
    <div class="container">
        <h1>🌡️ ROS2 Temperature Monitor</h1>
        
        <div id="status" class="status disconnected">
            🔌 Connecting to data stream...
        </div>
        
        <div class="card">
            <h2>Current Reading</h2>
            <div class="temp-display" id="temperature">--°C</div>
            <div class="humidity-display" id="humidity">--%</div>
            <div class="debug-info" id="debug-info">
                WebSocket Status: <span id="ws-status">Connecting...</span><br>
                Connection attempts: <span id="connection-attempts">0</span><br>
                Last message: <span id="last-message">None</span>
            </div>
        </div>
        
        <div class="card">
            <h2>Sensor Information</h2>
            <div class="info">
                <div class="info-item">
                    <div class="info-label">Sensor ID</div>
                    <div class="info-value" id="sensor-id">--</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Location</div>
                    <div class="info-value" id="location">--</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Last Update</div>
                    <div class="info-value" id="timestamp">--</div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h2>Temperature History</h2>
            <div class="chart">
                <canvas id="tempChart"></canvas>
            </div>
        </div>
    </div>

    <script>
        // WebSocket connection with retry logic
        let ws;
        let reconnectAttempts = 0;
        const maxReconnectAttempts = 10;
        const statusDiv = document.getElementById('status');
        const wsStatusSpan = document.getElementById('ws-status');
        const connectionAttemptsSpan = document.getElementById('connection-attempts');
        const lastMessageSpan = document.getElementById('last-message');
        
        // Chart setup
        const ctx = document.getElementById('tempChart').getContext('2d');
        const tempChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Temperature (°C)',
                    data: [],
                    borderColor: 'rgb(255, 255, 255)',
                    backgroundColor: 'rgba(255, 255, 255, 0.1)',
                    tension: 0.4,
                    borderWidth: 3,
                    pointBackgroundColor: 'rgb(255, 255, 255)',
                    pointBorderColor: 'rgb(255, 255, 255)',
                    pointRadius: 4
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        labels: {
                            color: 'white',
                            font: { size: 14 }
                        }
                    }
                },
                scales: {
                    x: {
                        ticks: { color: 'white' },
                        grid: { color: 'rgba(255,255,255,0.2)' }
                    },
                    y: {
                        ticks: { color: 'white' },
                        grid: { color: 'rgba(255,255,255,0.2)' },
                        beginAtZero: false
                    }
                }
            }
        });
        
        function connectWebSocket() {
            try {
                // Try different WebSocket URLs
                const wsUrls = [
                    'ws://localhost:8081',
                    'ws://127.0.0.1:8081',
                    `ws://${window.location.hostname}:8081`
                ];
                
                const wsUrl = wsUrls[reconnectAttempts % wsUrls.length];
                console.log(`Attempting to connect to: ${wsUrl}`);
                
                ws = new WebSocket(wsUrl);
                
                ws.onopen = function(event) {
                    statusDiv.className = 'status connected';
                    statusDiv.innerHTML = '✅ Connected to data stream';
                    wsStatusSpan.textContent = 'Connected';
                    reconnectAttempts = 0;
                    console.log('WebSocket connected to:', wsUrl);
                };
                
                ws.onclose = function(event) {
                    statusDiv.className = 'status disconnected';
                    wsStatusSpan.textContent = 'Disconnected';
                    console.log('WebSocket closed:', event.code, event.reason);
                    
                    if (reconnectAttempts < maxReconnectAttempts) {
                        const delay = Math.min(1000 * Math.pow(2, reconnectAttempts), 10000);
                        statusDiv.innerHTML = `⌛ Reconnecting in ${delay/1000}s... (${reconnectAttempts + 1}/${maxReconnectAttempts})`;
                        
                        setTimeout(() => {
                            reconnectAttempts++;
                            connectionAttemptsSpan.textContent = reconnectAttempts;
                            connectWebSocket();
                        }, delay);
                    } else {
                        statusDiv.innerHTML = '❌ Connection failed - Max retries reached';
                    }
                };
                
                ws.onerror = function(error) {
                    statusDiv.className = 'status disconnected';
                    statusDiv.innerHTML = '⚠️ Connection error';
                    wsStatusSpan.textContent = 'Error';
                    console.error('WebSocket error:', error);
                };
                
                ws.onmessage = function(event) {
                    try {
                        const data = JSON.parse(event.data);
                        console.log('Received data:', data);
                        lastMessageSpan.textContent = new Date().toLocaleTimeString();
                        
                        if (data.type === 'heartbeat_response') {
                            return; // Handle heartbeat responses
                        }
                        
                        // Update display
                        document.getElementById('temperature').textContent = data.temperature.toFixed(2) + '°' + data.unit;
                        document.getElementById('humidity').textContent = data.humidity.toFixed(1) + '%';
                        document.getElementById('sensor-id').textContent = data.sensor_id;
                        document.getElementById('location').textContent = data.location;
                        document.getElementById('timestamp').textContent = new Date(data.timestamp).toLocaleTimeString();
                        
                        // Update chart
                        const time = new Date(data.timestamp).toLocaleTimeString();
                        tempChart.data.labels.push(time);
                        tempChart.data.datasets[0].data.push(data.temperature);
                        
                        // Keep only last 20 data points
                        if (tempChart.data.labels.length > 20) {
                            tempChart.data.labels.shift();
                            tempChart.data.datasets[0].data.shift();
                        }
                        
                        tempChart.update('none');
                    } catch (error) {
                        console.error('Error parsing message:', error);
                    }
                };
            } catch (error) {
                console.error('Error creating WebSocket:', error);
            }
        }
        
        // Start connection
        connectWebSocket();
        
        // Send periodic heartbeat
        setInterval(() => {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({type: 'heartbeat'}));
            }
        }, 30000);
    </script>
</body>
</html>
        """
        return web.Response(text=html_content, content_type='text/html')

async def broadcast_loop(visualizer):
    """Continuous loop to broadcast data to WebSocket clients."""
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
            
        await asyncio.sleep(1.0)  # Broadcast every second

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    # Create ROS2 node
    visualizer = WebVisualizer()
    web_server = WebServer(visualizer)
    
    # Set up web application
    app = web.Application()
    app.router.add_get('/', web_server.serve_html)
    
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
            
            visualizer.get_logger().info('Web server started on http://0.0.0.0:8080')
            visualizer.get_logger().info('WebSocket server started on ws://0.0.0.0:8081')
            
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