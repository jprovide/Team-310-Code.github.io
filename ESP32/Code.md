## ESP32 Code ##

```cpp
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "Maggies iPhone 15 Pro Max";  // Your WiFi SSID
const char* password = "Mhlinde830";  // Your WiFi password

AsyncWebServer server(80);

// Global variables to store sensor data
String temperature = "N/A";
String humidity = "N/A";
String pressure = "N/A";

// Function to parse data received from UART
void parseData(String data) {
    int tempIndex = data.indexOf("T:") + 2;
    int humIndex = data.indexOf("H:") + 2;
    int presIndex = data.indexOf("P:") + 2;

    temperature = data.substring(tempIndex, data.indexOf(" ", tempIndex));
    humidity = data.substring(humIndex, data.indexOf(" ", humIndex));
    pressure = data.substring(presIndex, data.length());

    Serial.print("Parsed Temperature: "); Serial.println(temperature);
    Serial.print("Parsed Humidity: "); Serial.println(humidity);
    Serial.print("Parsed Pressure: "); Serial.println(pressure);
}

const char* index_html = 
"<!DOCTYPE HTML><html>\n"
"<head>\n"
"  <meta name='viewport' content='width=device-width, initial-scale=1'>\n"
"  <meta charset='UTF-8'>\n"
"  <script src='https://cdn.jsdelivr.net/npm/chart.js'></script>\n"
"  <style>\n"
"    body { background-color: #FFD1DC; display: flex; align-items: flex-start; }\n"
"    #leftPanel { width: 50%; padding: 10px; box-sizing: border-box; }\n"
"    #rightPanel { width: 50%; }\n"
"    canvas { max-width: 100%; height: auto !important; }\n"
"    .sensorValue { font-size: 1.5em; }\n"
"    .sensorLabel { font-weight: bold; }\n"
"  </style>\n"
"</head>\n"
"<body>\n"
"<div id='leftPanel'>\n"
"  <img src='data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAArUAAAK2CAIAAADfVvO4AAAgAElEQVR4Aey9Z3RbR5Y/ON/3036Y3f/+J/X0dprunp7ZObtnt6dn...' alt='Flora Forecast' style='width: 100px; height: auto;'>\n"
"  <h1>ESP32 Sensor Server</h1>\n"
"  <p class='sensorValue'>Current Temperature: <span id='displayTemp'></span>°C</p>\n"
"  <p class='sensorValue'>Current Humidity: <span id='displayHum'></span>%</p>\n"
"  <p class='sensorValue'>Current Pressure: <span id='displayPress'></span> hPa</p>\n"
"</div>\n"
"<div id='rightPanel'>\n"
"  <canvas id='tempChart'></canvas>\n"
"  <canvas id='humChart'></canvas>\n"
"  <canvas id='pressChart'></canvas>\n"
"</div>\n"
"  <script>\n"
"    const ctxTemp = document.getElementById('tempChart').getContext('2d');\n"
"    const ctxHum = document.getElementById('humChart').getContext('2d');\n"
"    const ctxPress = document.getElementById('pressChart').getContext('2d');\n"
"    const tempChart = new Chart(ctxTemp, {\n"
"      type: 'line',\n"
"      data: { labels: [], datasets: [{ label: 'Temperature (°C)', data: [], borderColor: 'red', fill: false }] },\n"
"      options: { scales: { y: { beginAtZero: true } } }\n"
"    });\n"
"    const humChart = new Chart(ctxHum, {\n"
"      type: 'line',\n"
"      data: { labels: [], datasets: [{ label: 'Humidity (%)', data: [], borderColor: 'blue', fill: false }] },\n"
"      options: { scales: { y: { beginAtZero: true } } }\n"
"    });\n"
"    const pressChart = new Chart(ctxPress, {\n"
"      type: 'line',\n"
"      data: { labels: [], datasets: [{ label: 'Pressure (hPa)', data: [], borderColor: 'green', fill: false }] },\n"
"      options: { scales: { y: { beginAtZero: true } } }\n"
"    });\n"
"    function fetchData() {\n"
"      fetch('/data')\n"
"        .then(response => response.json())\n"
"        .then(data => {\n"
"          const now = new Date();\n"
"          const hours = now.getHours() % 12 || 12;\n"
"          const minutes = now.getMinutes().toString().padStart(2, '0');\n"
"          const label = hours + ':' + minutes + ' ' + (now.getHours() >= 12 ? 'PM' : 'AM');\n"
"          document.getElementById('displayTemp').textContent = data.temperature;\n"
"          document.getElementById('displayHum').textContent = data.humidity;\n"
"          document.getElementById('displayPress').textContent = data.pressure;\n"
"          tempChart.data.labels.push(label);\n"
"          tempChart.data.datasets[0].data.push(data.temperature);\n"
"          tempChart.update();\n"
"          humChart.data.labels.push(label);\n"
"          humChart.data.datasets[0].data.push(data.humidity);\n"
"          humChart.update();\n"
"          pressChart.data.labels.push(label);\n"
"          pressChart.data.datasets[0].data.push(data.pressure);\n"
"          pressChart.update();\n"
"        })\n"
"        .catch(error => console.error('Error fetching data:', error));\n"
"    }\n"
"    setInterval(fetchData, 1000); // Update every second\n"
"  </script>\n"
"</body>\n"
"</html>\n";

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Configure UART2

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html, NULL);
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    String data = "{\"temperature\":\"" + temperature + "\",\"humidity\":\"" + humidity + "\",\"pressure\":\"" + pressure + "\"}";
    request->send(200, "application/json", data);
  });

  server.begin();
}

void loop() {
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    Serial.print("Received data: "); Serial.println(data); // Debug received data
    parseData(data);
  }
}
```

