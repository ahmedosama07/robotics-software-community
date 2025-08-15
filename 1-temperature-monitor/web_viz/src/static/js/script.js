// Global variables 
let ws; let reconnectAttempts = 0;
const maxReconnectAttempts = 10;
let debugMode = true; // Enable debug logging

// Charts
let liveChart, historyChart, hourlyChart;

// Data storage
let sensorList = new Set();
let historicalData = [];
let statisticsData = [];

// Debug logging
function debugLog(message, data = null) {
    if (debugMode) {
        console.log('[DEBUG]', message, data);
        const debugDiv = document.getElementById('debug-info');
        if (debugDiv) {
            debugDiv.style.display = 'block';
            debugDiv.innerHTML = `[${new Date().toLocaleTimeString()}] ${message}` +
                (data ? `<br/><pre>${JSON.stringify(data, null, 2).substring(0, 200)}</pre>` : '');
        }
    }
}

// Show error message
function showError(message, containerId) {
    const container = document.getElementById(containerId);
    if (container) {
        container.innerHTML = `<div class="error-message">❌ ${message}</div>`;
    }
}

// Tab management
function showTab(tabName) {
    debugLog(`Switching to tab: ${tabName}`);

    // Hide all tab panes
    document.querySelectorAll('.tab-pane').forEach(pane => {
        pane.classList.remove('active');
    });

    // Remove active class from all tabs
    document.querySelectorAll('.tab').forEach(tab => {
        tab.classList.remove('active');
    });

    // Show selected tab pane
    const tabPane = document.getElementById(tabName + '-tab');
    if (tabPane) {
        tabPane.classList.add('active');
    }

    // Add active class to selected tab
    event.target.classList.add('active');

    // Load data for specific tabs
    switch (tabName) {
        case 'history':
            setTimeout(() => loadHistoricalData(), 100);
            break;
        case 'statistics':
            setTimeout(() => loadStatistics(), 100);
            break;
        case 'trends':
            setTimeout(() => loadTrends(), 100);
            break;
    }
}

// Initialize charts
function initializeCharts() {
    debugLog('Initializing charts');

    // Live chart
    const liveCtx = document.getElementById('liveChart');
    if (liveCtx) {
        liveChart = new Chart(liveCtx.getContext('2d'), {
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
                        labels: { color: 'white', font: { size: 14 } }
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
    }

    // History chart
    const historyCtx = document.getElementById('historyChart');
    if (historyCtx) {
        historyChart = new Chart(historyCtx.getContext('2d'), {
            type: 'line',
            data: {
                labels: [],
                datasets: []
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        labels: { color: 'white', font: { size: 12 } }
                    }
                },
                scales: {
                    x: {
                        ticks: { color: 'white' },
                        grid: { color: 'rgba(255,255,255,0.2)' }
                    },
                    y: {
                        ticks: { color: 'white' },
                        grid: { color: 'rgba(255,255,255,0.2)' }
                    }
                }
            }
        });
    }

    // Hourly chart
    const hourlyCtx = document.getElementById('hourlyChart');
    if (hourlyCtx) {
        hourlyChart = new Chart(hourlyCtx.getContext('2d'), {
            type: 'bar',
            data: {
                labels: [],
                datasets: []
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        labels: { color: 'white', font: { size: 12 } }
                    }
                },
                scales: {
                    x: {
                        ticks: { color: 'white' },
                        grid: { color: 'rgba(255,255,255,0.2)' }
                    },
                    y: {
                        ticks: { color: 'white' },
                        grid: { color: 'rgba(255,255,255,0.2)' }
                    }
                }
            }
        });
    }
}

// WebSocket connection
function connectWebSocket() {
    try {
        const hostname = window.location.hostname;
        const wsUrl = `ws://${hostname}:8081`;
        debugLog(`Connecting to WebSocket: ${wsUrl}`);
        ws = new WebSocket(wsUrl);

        ws.onopen = function (event) {
            document.getElementById('status').className = 'status connected';
            document.getElementById('status').innerHTML = '✅ Connected to enhanced data stream';
            reconnectAttempts = 0;
            debugLog('WebSocket connected successfully');
        };

        ws.onclose = function (event) {
            document.getElementById('status').className = 'status disconnected';
            document.getElementById('status').innerHTML = '❌ Disconnected from data stream';
            debugLog('WebSocket disconnected', { code: event.code, reason: event.reason });

            if (reconnectAttempts < maxReconnectAttempts) {
                const delay = Math.min(1000 * Math.pow(2, reconnectAttempts), 10000);
                debugLog(`Attempting reconnect in ${delay}ms (attempt ${reconnectAttempts + 1})`);
                setTimeout(() => {
                    reconnectAttempts++;
                    connectWebSocket();
                }, delay);
            }
        };

        ws.onerror = function (error) {
            document.getElementById('status').className = 'status disconnected';
            document.getElementById('status').innerHTML = '⚠️ Connection error';
            debugLog('WebSocket error:', error);
        };

        ws.onmessage = function (event) {
            try {
                const data = JSON.parse(event.data);
                debugLog('Received WebSocket message:', { type: data.type, dataSize: JSON.stringify(data).length });
                handleWebSocketMessage(data);
            } catch (error) {
                console.error('Error parsing message:', error);
                debugLog('Failed to parse message:', event.data);
            }
        };
    } catch (error) {
        debugLog('Error creating WebSocket:', error);
    }
}

// Handle WebSocket messages
function handleWebSocketMessage(data) {
    switch (data.type) {
        case 'welcome':
            debugLog('Connected to enhanced temperature monitor', data);
            break;

        case 'live_data':
            updateLiveData(data);
            break;

        case 'recent_data':
            debugLog('Received recent data:', { count: data.data ? data.data.length : 0 });
            updateHistoricalTable(data.data);
            break;

        case 'statistics':
            debugLog('Received statistics:', { count: data.data ? data.data.length : 0 });
            updateStatistics(data.data);
            break;

        case 'hourly_averages':
            debugLog('Received hourly averages:', { count: data.data ? data.data.length : 0 });
            updateHourlyChart(data.data);
            break;

        case 'trends':
            debugLog('Received trends:', data.data);
            updateTrends(data.data);
            break;

        default:
            debugLog('Unknown message type:', data.type);
    }
}

// Update live data display
function updateLiveData(data) {
    try {
        // Update displays
        document.getElementById('temperature').textContent = data.temperature.toFixed(2) + '°' + data.unit;
        document.getElementById('humidity').textContent = data.humidity.toFixed(1) + '%';
        document.getElementById('sensor-id').textContent = data.sensor_id;
        document.getElementById('location').textContent = data.location;
        document.getElementById('timestamp').textContent = new Date(data.timestamp).toLocaleTimeString();

        // Add to sensor list
        sensorList.add(data.sensor_id);
        updateSensorSelects();

        // Update live chart
        if (liveChart) {
            const time = new Date(data.timestamp).toLocaleTimeString();
            liveChart.data.labels.push(time);
            liveChart.data.datasets[0].data.push(data.temperature);

            // Keep only last 20 data points
            if (liveChart.data.labels.length > 20) {
                liveChart.data.labels.shift();
                liveChart.data.datasets[0].data.shift();
            }

            liveChart.update('none');
        }

        debugLog('Updated live data:', { temp: data.temperature, sensor: data.sensor_id });
    } catch (error) {
        debugLog('Error updating live data:', error);
    }
}

// Update sensor select dropdowns
function updateSensorSelects() {
    const selects = ['history-sensor', 'trends-sensor'];
    selects.forEach(selectId => {
        const select = document.getElementById(selectId);
        if (select) {
            const currentValue = select.value;

            // Clear existing options except "All Sensors"
            while (select.children.length > 1) {
                select.removeChild(select.lastChild);
            }

            // Add sensor options
            sensorList.forEach(sensor => {
                const option = document.createElement('option');
                option.value = sensor;
                option.textContent = sensor;
                if (sensor === currentValue) {
                    option.selected = true;
                }
                select.appendChild(option);
            });
        }
    });
}

// Send WebSocket request with timeout
function sendWebSocketRequest(requestData, timeoutMs = 10000) {
    return new Promise((resolve, reject) => {
        if (!ws || ws.readyState !== WebSocket.OPEN) {
            debugLog('WebSocket not connected, cannot send request');
            reject(new Error('WebSocket not connected'));
            return;
        }
        debugLog('Sending WebSocket request:', requestData);

        // Set up timeout
        const timeout = setTimeout(() => {
            debugLog('WebSocket request timeout');
            reject(new Error('Request timeout'));
        }, timeoutMs);

        // Send request
        try {
            ws.send(JSON.stringify(requestData));

            // For now, we'll resolve immediately since we handle responses in handleWebSocketMessage
            // In a production app, you'd want to implement proper request/response correlation
            clearTimeout(timeout);
            resolve();
        } catch (error) {
            clearTimeout(timeout);
            reject(error);
        }
    });
}

// Load historical data
async function loadHistoricalData() {
    try {
        const hours = parseInt(document.getElementById('history-hours').value);
        const sensorId = document.getElementById('history-sensor').value;
        debugLog('Loading historical data...', { hours, sensorId });

        // Clear existing data
        document.getElementById('history-table-container').innerHTML = '<div class="loading">Loading historical data...</div>';

        // Request recent readings
        await sendWebSocketRequest({
            type: 'get_recent_data',
            hours: hours,
            limit: 200
        });

        // Request hourly averages
        await sendWebSocketRequest({
            type: 'get_hourly_averages',
            sensor_id: sensorId || null,
            hours: hours
        });

    } catch (error) {
        debugLog('Error loading historical data:', error);
        showError('Failed to load historical data: ' + error.message, 'history-table-container');
    }
}

// Update historical table
function updateHistoricalTable(data) {
    const container = document.getElementById('history-table-container');
    if (!data || data.length === 0) {
        container.innerHTML = '<div class="loading">No historical data available</div>';
        debugLog('No historical data received');
        return;
    }

    debugLog('Updating historical table with data:', { count: data.length });

    let html = `
    <table class="data-table">
        <thead>
            <tr>
                <th>Timestamp</th>
                <th>Sensor ID</th>
                <th>Temperature</th>
                <th>Humidity</th>
                <th>Location</th>
            </tr>
        </thead>
        <tbody>
`;

    data.slice(0, 50).forEach(reading => {
        html += `
        <tr>
            <td>${new Date(reading.timestamp).toLocaleString()}</td>
            <td>${reading.sensor_id}</td>
            <td>${reading.temperature.toFixed(2)}°${reading.unit}</td>
            <td>${reading.humidity.toFixed(1)}%</td>
            <td>${reading.location}</td>
        </tr>
    `;
    });

    html += '</tbody></table>';
    container.innerHTML = html;

    // Update history chart
    updateHistoryChart(data);
}

// Update history chart
function updateHistoryChart(data) {
    if (!historyChart || !data || data.length === 0) {
        debugLog('Cannot update history chart - missing chart or data');
        return;
    }
    const sensorData = {};

    // Group data by sensor
    data.forEach(reading => {
        if (!sensorData[reading.sensor_id]) {
            sensorData[reading.sensor_id] = {
                labels: [],
                data: []
            };
        }

        sensorData[reading.sensor_id].labels.push(
            new Date(reading.timestamp).toLocaleTimeString()
        );
        sensorData[reading.sensor_id].data.push(reading.temperature);
    });

    // Prepare chart datasets
    const colors = [
        'rgb(255, 99, 132)',
        'rgb(54, 162, 235)',
        'rgb(255, 205, 86)',
        'rgb(75, 192, 192)',
        'rgb(153, 102, 255)'
    ];

    const datasets = [];
    let colorIndex = 0;

    Object.keys(sensorData).forEach(sensorId => {
        const sensorReadings = sensorData[sensorId];
        datasets.push({
            label: sensorId,
            data: sensorReadings.data,
            borderColor: colors[colorIndex % colors.length],
            backgroundColor: colors[colorIndex % colors.length] + '20',
            tension: 0.4,
            borderWidth: 2
        });
        colorIndex++;
    });

    // Update chart
    const allLabels = data.map(reading =>
        new Date(reading.timestamp).toLocaleTimeString()
    ).reverse();

    historyChart.data.labels = allLabels;
    historyChart.data.datasets = datasets;
    historyChart.update();

    debugLog('Updated history chart with datasets:', { count: datasets.length });
}

// Update hourly chart
function updateHourlyChart(data) {
    if (!hourlyChart) {
        debugLog('Hourly chart not initialized');
        return;
    }

    if (!data || data.length === 0) {
        debugLog('No hourly averages data received');
        return;
    }

    debugLog('Updating hourly chart with data:', { count: data.length });

    const sensorData = {};

    // Group data by sensor
    data.forEach(avg => {
        if (!sensorData[avg.sensor_id]) {
            sensorData[avg.sensor_id] = {
                labels: [],
                data: []
            };
        }

        sensorData[avg.sensor_id].labels.push(
            new Date(avg.hour).toLocaleString()
        );
        sensorData[avg.sensor_id].data.push(parseFloat(avg.avg_temp));
    });

    // Prepare chart datasets
    const colors = [
        'rgba(255, 99, 132, 0.8)',
        'rgba(54, 162, 235, 0.8)',
        'rgba(255, 205, 86, 0.8)',
        'rgba(75, 192, 192, 0.8)',
        'rgba(153, 102, 255, 0.8)'
    ];

    const datasets = [];
    let colorIndex = 0;

    Object.keys(sensorData).forEach(sensorId => {
        const sensorReadings = sensorData[sensorId];
        datasets.push({
            label: sensorId,
            data: sensorReadings.data,
            backgroundColor: colors[colorIndex % colors.length],
            borderColor: colors[colorIndex % colors.length],
            borderWidth: 1
        });
        colorIndex++;
    });

    // Use labels from data (should be consistent across sensors)
    const labels = data.map(avg =>
        new Date(avg.hour).toLocaleString()
    ).reverse();

    hourlyChart.data.labels = labels;
    hourlyChart.data.datasets = datasets;
    hourlyChart.update();

    debugLog('Updated hourly chart with datasets:', { count: datasets.length });
}

// Load statistics
async function loadStatistics() {
    try {
        debugLog('Loading statistics...');
        document.getElementById('statistics-content').innerHTML = '<div class="loading">Loading statistics...</div>';
        await sendWebSocketRequest({
            type: 'get_statistics'
        });

    } catch (error) {
        debugLog('Error loading statistics:', error);
        showError('Failed to load statistics: ' + error.message, 'statistics-content');
    }
}

// Update statistics display
function updateStatistics(data) {
    const container = document.getElementById('statistics-content');
    if (!data || data.length === 0) {
        container.innerHTML = '<div class="loading">No statistics available</div>';
        debugLog('No statistics data received');
        return;
    }

    debugLog('Updating statistics display with data:', { count: data.length });

    let html = '<div class="grid">';

    data.forEach(stat => {
        html += `
        <div class="card">
            <h3>📡 ${stat.sensor_id}</h3>
            <div class="info-grid">
                <div class="info-item">
                    <div class="info-label">Location</div>
                    <div class="info-value">${stat.location}</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Readings</div>
                    <div class="info-value">${stat.reading_count}</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Avg Temp</div>
                    <div class="info-value">${parseFloat(stat.avg_temperature).toFixed(2)}°C</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Min Temp</div>
                    <div class="info-value">${parseFloat(stat.min_temperature).toFixed(2)}°C</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Max Temp</div>
                    <div class="info-value">${parseFloat(stat.max_temperature).toFixed(2)}°C</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Std Dev</div>
                    <div class="info-value">${parseFloat(stat.temp_stddev || 0).toFixed(2)}°C</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Avg Humidity</div>
                    <div class="info-value">${parseFloat(stat.avg_humidity).toFixed(1)}%</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Active Since</div>
                    <div class="info-value">${new Date(stat.first_reading).toLocaleString()}</div>
                </div>
            </div>
        </div>
    `;
    });

    html += '</div>';
    container.innerHTML = html;
}

// Load trends
async function loadTrends() {
    try {
        const hours = parseInt(document.getElementById('trends-hours').value);
        const sensorId = document.getElementById('trends-sensor').value;

        debugLog('Loading trends...', { hours, sensorId });
        document.getElementById('trends-content').innerHTML = '<div class="loading">Loading trend analysis...</div>';

        await sendWebSocketRequest({
            type: 'get_trends',
            sensor_id: sensorId || null,
            hours: hours
        });

    } catch (error) {
        debugLog('Error loading trends:', error);
        showError('Failed to load trends: ' + error.message, 'trends-content');
    }
}

// Update trends display
function updateTrends(data) {
    const container = document.getElementById('trends-content');
    if (!data || Object.keys(data).length === 0) {
        container.innerHTML = '<div class="loading">No trend data available</div>';
        debugLog('No trends data received');
        return;
    }

    debugLog('Updating trends display with data:', data);

    let trendText = 'Stable';
    let trendClass = 'trend-stable';

    if (data.rising_percentage > 40) {
        trendText = 'Rising';
        trendClass = 'trend-rising';
    } else if (data.falling_percentage > 40) {
        trendText = 'Falling';
        trendClass = 'trend-falling';
    }

    const html = `
    <div class="grid">
        <div class="card">
            <h3>📈 Temperature Analysis</h3>
            <div class="info-grid">
                <div class="info-item">
                    <div class="info-label">Total Readings</div>
                    <div class="info-value">${data.total_readings}</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Average</div>
                    <div class="info-value">${parseFloat(data.avg_temp).toFixed(2)}°C</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Range</div>
                    <div class="info-value">${parseFloat(data.min_temp).toFixed(2)}°C - ${parseFloat(data.max_temp).toFixed(2)}°C</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Variability</div>
                    <div class="info-value">±${parseFloat(data.temp_stddev || 0).toFixed(2)}°C</div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h3>📊 Trend Analysis</h3>
            <div class="info-grid">
                <div class="info-item">
                    <div class="info-label">Overall Trend</div>
                    <div class="info-value">
                        ${trendText}
                        <span class="trend-indicator ${trendClass}">${trendText.toUpperCase()}</span>
                    </div>
                </div>
                <div class="info-item">
                    <div class="info-label">Rising</div>
                    <div class="info-value">${data.rising_percentage}% (${data.rising_count})</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Falling</div>
                    <div class="info-value">${data.falling_percentage}% (${data.falling_count})</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Stable</div>
                    <div class="info-value">${data.stable_percentage}% (${data.stable_count})</div>
                </div>
            </div>
        </div>
    </div>
`;

    container.innerHTML = html;
}

// Export functions
function exportHistoricalData() {
    debugLog('Exporting historical data to CSV');
    
    // Get the current historical data from the table
    const tableContainer = document.getElementById('history-table-container');
    const table = tableContainer.querySelector('.data-table');
    
    if (!table) {
        alert('No historical data available to export');
        return;
    }

    // Convert table to CSV
    const csv = convertTableToCSV(table);
    debugLog('Generated CSV data', csv);

    // Create download link
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `temperature_data_${new Date().toISOString().slice(0, 10)}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}

function exportStatistics() {
    debugLog('Exporting statistics data to CSV');
    
    // Get the current statistics data from the container
    const statsContainer = document.getElementById('statistics-content');
    const statsCards = statsContainer.querySelectorAll('.card');
    
    if (!statsCards || statsCards.length === 0) {
        alert('No statistics data available to export');
        return;
    }

    // Convert statistics data to CSV
    const csv = convertStatisticsToCSV(statsCards);
    debugLog('Generated CSV data', csv);

    // Create download link
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `sensor_statistics_${new Date().toISOString().slice(0, 10)}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}

// Add helper function to convert table to CSV
function convertTableToCSV(table) {
    const rows = table.querySelectorAll('tr');
    let csv = [];
    
    for (let i = 0; i < rows.length; i++) {
        let row = [], cols = rows[i].querySelectorAll('td, th');
        
        for (let j = 0; j < cols.length; j++) {
            // Clean inner text and handle commas
            let text = cols[j].innerText.replace(/(\r\n|\n|\r)/gm, '').replace(/(\s\s)/gm, ' ');
            text = text.replace(/"/g, '""');
            row.push(`"${text}"`);
        }
        csv.push(row.join(','));
    }
    
    return csv.join('\n');
}

// Add helper function to convert statistics to CSV
function convertStatisticsToCSV(statsCards) {
    let csv = ['Sensor ID,Location,Readings,Avg Temp,Min Temp,Max Temp,Std Dev,Avg Humidity,First Reading,Last Reading'];
    
    statsCards.forEach(card => {
        const sensorId = card.querySelector('h3').textContent.replace('📡 ', '');
        const infoItems = card.querySelectorAll('.info-item');
        
        let row = [sensorId];
        
        infoItems.forEach(item => {
            const value = item.querySelector('.info-value').textContent;
            row.push(value);
        });
        
        csv.push(row.join(','));
    });
    
    return csv.join('\n');
}

// Initialize everything
document.addEventListener('DOMContentLoaded', function () {
    debugLog('Page loaded, initializing...');
    initializeCharts();
    connectWebSocket();
    // Set up event listeners
    document.querySelectorAll('.tab').forEach(tab => {
        tab.addEventListener('click', function () {
            showTab(this.getAttribute('data-tab'));
        });
    });

    document.getElementById('refresh-history').addEventListener('click', loadHistoricalData);
    document.getElementById('export-history').addEventListener('click', exportHistoricalData);
    document.getElementById('refresh-stats').addEventListener('click', loadStatistics);
    document.getElementById('export-stats').addEventListener('click', exportStatistics);
    document.getElementById('analyze-trends').addEventListener('click', loadTrends);

    // Send periodic heartbeat and data requests
    setInterval(() => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'heartbeat' }));
        }
    }, 30000);

    debugLog('Initialization complete');
});

// Handle page visibility changes
document.addEventListener('visibilitychange', function () {
    if (document.visibilityState === 'visible' && (!ws || ws.readyState !== WebSocket.OPEN)) {
        debugLog('Page became visible, reconnecting WebSocket...');
        connectWebSocket();
    }
});