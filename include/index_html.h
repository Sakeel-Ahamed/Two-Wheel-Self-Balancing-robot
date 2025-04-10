#ifndef INDEX_HTML_H
#define INDEX_HTML_H

#include <Arduino.h>

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP32 Real-Time Tracking</title>
  <style>
    body { text-align: center; font-family: Arial, sans-serif; }
    #canvas { border: 1px solid #000; margin-top: 20px; }
    #ldInfo { margin-top: 10px; font-size: 16px; }
  </style>
</head>
<body>
  <h1>ESP32 Real-Time Tracking</h1>
  <canvas id="canvas" width="400" height="400"></canvas>
  <div id="ldInfo">LD06 Info: N/A</div>
  <script>
    var canvas = document.getElementById('canvas');
    var ctx = canvas.getContext('2d');
    var offsetX = canvas.width / 2, offsetY = canvas.height / 2;
    var path = [];
    
    // Create WebSocket connection to the ESP32
    var ws = new WebSocket('ws://' + window.location.hostname + '/ws');
    ws.onopen = function() {
      console.log("WebSocket connected");
    };
    ws.onmessage = function(event) {
  // Check if the message is a coordinate message
  if (event.data.startsWith("<") && event.data.endsWith(">")) {
    // Remove the angle brackets
    var dataStr = event.data.substring(1, event.data.length - 1);
    // Split by comma
    var parts = dataStr.split(",");
    // Example parts: ["D:0.00", "Y:34.43", "X:1.86", "Y:1.24", "V:0.00"]
    var posX = null, posY = null;
    parts.forEach(function(part) {
      var trimmed = part.trim();
      if (trimmed.startsWith("X:")) {
        posX = parseFloat(trimmed.substring(2));
      } else if (trimmed.startsWith("Y:") && posX != null) { 
        // Assuming the second Y is the position Y. You might want to use a different label if available.
        posY = parseFloat(trimmed.substring(2));
      }
    });
    if (posX !== null && posY !== null) {
      path.push({x: posX, y: posY});
      drawPath();
    }
  } else if (event.data.startsWith("LD:")) {
    var ldData = event.data.substring(3);
    var parts = ldData.split(",");
    if (parts.length === 3) {
      document.getElementById("ldInfo").innerText = "Forward Avg: " + parts[0] + " mm, Backward Avg: " + parts[1] + " mm, Decision: " + parts[2];
    }
  }
};
    
    function drawPath() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      
      // Draw reference axes
      ctx.beginPath();
      ctx.moveTo(offsetX, 0);
      ctx.lineTo(offsetX, canvas.height);
      ctx.moveTo(0, offsetY);
      ctx.lineTo(canvas.width, offsetY);
      ctx.strokeStyle = "#ccc";
      ctx.stroke();
      
      // Draw the path (scaling coordinates by 20 for visibility)
      ctx.beginPath();
      for (var i = 0; i < path.length; i++) {
        var px = offsetX + path[i].x * 20;
        var py = offsetY - path[i].y * 20;
        if (i === 0)
          ctx.moveTo(px, py);
        else
          ctx.lineTo(px, py);
      }
      ctx.strokeStyle = "red";
      ctx.lineWidth = 2;
      ctx.stroke();
      
      // Draw current position as a blue circle
      if (path.length > 0) {
        var current = path[path.length - 1];
        var cx = offsetX + current.x * 20;
        var cy = offsetY - current.y * 20;
        ctx.beginPath();
        ctx.arc(cx, cy, 5, 0, 2 * Math.PI);
        ctx.fillStyle = "blue";
        ctx.fill();
      }
    }
  </script>
</body>
</html>
)rawliteral";

#endif  // INDEX_HTML_H
