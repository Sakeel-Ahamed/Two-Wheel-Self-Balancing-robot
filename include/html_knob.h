#ifndef HTML_KNOB_H
#define HTML_KNOB_H

const char* htmlKnobPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 PID Sliders</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            margin: 0;
            padding: 0;
        }
        .container {
            display: flex;
            justify-content: space-between;
            align-items: flex-start;
            padding: 20px;
        }
        .column {
            width: 45%;
        }
        .column h2 {
            text-align: center;
            font-size: 1.5em;
        }
        .slider-container {
            margin: 20px 0;
        }
        .slider-label {
            margin-bottom: 5px;
            font-size: 1.2em;
        }
        input[type="range"] {
            width: 100%;
            -webkit-appearance: none;
            appearance: none;
            height: 10px;
            background: #ddd;
            border-radius: 5px;
            outline: none;
        }
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            background: #333;
            border-radius: 50%;
            cursor: pointer;
        }
        .value-display {
            margin-top: 10px;
            font-size: 1em;
        }
    </style>
    <script>
        function updateValue(id, value) {
            // Convert the slider value to a number with 3 decimal places
            const preciseValue = (value / 1000).toFixed(3);
            document.getElementById(id).innerText = preciseValue;

            // Send the precise value to the ESP32
            fetch('/set_value', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: id + '=' + encodeURIComponent(preciseValue)
            }).then(response => response.text())
              .then(message => console.log("Response: " + message))
              .catch(error => console.error("Error:", error));
        }
    </script>
</head>
<body>
    <h1>ESP32 PID Sliders</h1>
    <div class="container">
        <!-- Gyro PID Column -->
        <div class="column">
            <h2>Gyro PID</h2>
            <div class="slider-container">
                <div class="slider-label">Gyro P</div>
                <input type="range" min="0" max="15000" step="1" value="1500" oninput="updateValue('gyroP', this.value)">
                <div class="value-display">Value: <span id="gyroP">1.500</span></div>
            </div>
            <div class="slider-container">
                <div class="slider-label">Gyro I</div>
                <input type="range" min="0" max="15000" step="1" value="9500" oninput="updateValue('gyroI', this.value)">
                <div class="value-display">Value: <span id="gyroI">9.500</span></div>
            </div>
            <div class="slider-container">
                <div class="slider-label">Gyro D</div>
                <input type="range" min="0" max="15000" step="1" value="0165" oninput="updateValue('gyroD', this.value)">
                <div class="value-display">Value: <span id="gyroD">0.165</span></div>
            </div>
        </div>

        <!-- Motor PID Column -->
        <div class="column">
            <h2>Motor PID</h2>
            <div class="slider-container">
                <div class="slider-label">Motor P</div>
                <input type="range" min="0" max="15000" step="1" value="0500" oninput="updateValue('motorP', this.value)">
                <div class="value-display">Value: <span id="motorP">0.500</span></div>
            </div>
            <div class="slider-container">
                <div class="slider-label">Motor I</div>
                <input type="range" min="0" max="15000" step="1" value="10750" oninput="updateValue('motorI', this.value)">
                <div class="value-display">Value: <span id="motorI">10.75</span></div>
            </div>
            <div class="slider-container">
                <div class="slider-label">Motor D</div>
                <input type="range" min="0" max="15000" step="1" value="0010" oninput="updateValue('motorD', this.value)">
                <div class="value-display">Value: <span id="motorD">0.010</span></div>
            </div>
        </div>
    </div>
</body>
</html>
)rawliteral";

#endif // HTML_KNOB_H
