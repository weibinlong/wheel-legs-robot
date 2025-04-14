// -----------------------------------------------------------------------------
// Copyright (c) 2024 Mu Shibo
// Modify by Tommy
// -----------------------------------------------------------------------------

#include <WiFi.h>

/***********html&javascript网页构建**********/
const char basic_web[] PROGMEM = R"=====(

<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>legged wheel robot web ctrl</title>
  <style>
    h2{width: auto;height: 60px;line-height: 60px;text-align: center;font-family: 等线;color: white;background-color:cornflowerblue;border-radius: 12px;}
    input{width: 160px;height: 30px;margin: 0px;}
    .view{width: 140px;height: 30px;padding: 0px;margin: 15px auto;vertical-align:middle;}
    .dir{font-size: 15px;width: 100px;height: 60px;text-align: center;border-radius: 12px;background-color: white;color: cornflowerblue;border: 3px solid cornflowerblue;padding: 0px;transition: all 0.3s;}
    input[type='checkbox'].switch{outline: none;appearance: none;-webkit-appearance: none;-moz-appearance: none;position: relative;width: 40px;height: 20px;background: #ccc;border-radius: 10px;transition: border-color .3s, background-color .3s;margin: 0px 20px 0px 0px;}
    input[type='checkbox'].switch::after{content: '';display: inline-block;width: 1rem;height:1rem;border-radius: 50%;background: #fff;box-shadow: 0, 0, 2px, #999;transition:.4s;top: 2px;position: absolute;left: 2px;}
    input[type='checkbox'].switch:checked{background: rgb(78, 78, 240);}
    input[type='checkbox'].switch:checked::after{content: '';position: absolute;left: 55%;top: 2px;}
    *{-webkit-touch-callout:none;-webkit-user-select:none;-khtml-user-select:none;-moz-user-select:none;-ms-user-select:none;user-select:none;}
    .row{display: flex;justify-content: center;clear: both;}
    .columnLateral{float: left;width: 15%;min-width: 300px;}
    .buttons{width: 300px;height: 180px;padding: 10px;margin: 10px auto;position: relative;}
    #jump{display: inline-block;}
  </style>
</head>

<!-- onload 事件在页面载入完成后立即触发 -->
<body onload="javascript:socket_init()">
    <h2>WL-PRO WiFi遥控模式</h2>
    <div class="view" style="display:none">
        <input type="checkbox" id="stable" class="switch" onclick="is_stable()" style="vertical-align:middle">Robot Go!</input>
    </div>
    <center>
        <!-- Example of two JoyStick integrated in the page structure -->
        <div class="row">
            <div class="columnLateral">
                <div id="joy1Div" style="width:200px;height:200px;margin:10px"></div>
                <!-- 新增显示 joy1 的 x 和 y 值 -->
                <div id="joy1Info">
                    <span>Joy_x: <span id="joy1XValue">0</span></span>
                    <br>
                    <span>Joy_y: <span id="joy1YValue">0</span></span>
                </div>
            </div>
           
            <div class="columnLateral">
                <div id="joy2Div" style="width:200px;height:200px;margin:10px"></div>
                <!-- 新增显示 joy2 的 x 和 y 值 -->
                <div id="joy2Info">
                    <span>Roll: <span id="joy2XValue">0</span></span>
                    <br>
                    <span>Height: <span id="joy2YValue">32</span></span>
                </div>
            </div>
        </div>

        
        <div class="buttons">
            <button class="dir" id="jump">Jump</button>
        </div>
    </center>
    
    <script>
        var socket; // socket通信
        var g_roll=0; g_h=38; 
        var g_stable = 0; 
        var joyX = 0;
        var joyY = 0;
        // socket_init在页面载入完成后触发
        function socket_init() {
            // 初始化websocket客户端
            //socket = new WebSocket('ws://' + '192.168.4.1' + ':81/'); // AP模式
            socket = new WebSocket('ws://' + window.location.hostname + ':81/'); // sta模式
        }

        function send_data() {
            var data = {
                'roll': g_roll,
                'height': g_h,
                'stable': g_stable,
                'mode': 'basic',
                'dir': "stop",
                'joy_y': joyY,
                'joy_x': joyX
            };
            console.log(data);
            socket.send(JSON.stringify(data));
        }

        function is_stable() {
            var obj = document.getElementById("stable");
            if(obj.checked) {
                // alert("is_stable checked");
                g_stable = 1;
            } else {
                // alert("is_stable unchecked");
                g_stable = 0;
            }
            send_data();
        }

        // 绑定按钮
        var buttons = document.getElementsByClassName("dir");
        for(i=0;i<buttons.length;i++) {
            buttons[i].addEventListener("mousedown",move,true);
            buttons[i].addEventListener("mouseup",stop,true);
            buttons[i].addEventListener("touchstart",move,true);
            buttons[i].addEventListener("touchend",stop,true);
        }

        function move() {
            this.style = "background-color: cornflowerblue; color: white;";
            var data = {
                'dir': this.id,
                'mode': 'basic',
                'roll': g_roll,
                'height': g_h,
                'stable': g_stable,
                'joy_x': joyX,
                'joy_y': joyY
            };
            console.log(data);
            socket.send(JSON.stringify(data));
           
        }

        function stop() {
            this.style = "background-color: white; color: cornflowerblue;";
            var data = {
                'dir': "stop",
                'mode': 'basic',
                'roll': g_roll,
                'height': g_h,
                'stable': g_stable,
                'joy_x': joyX,
                'joy_y': joyY
            };
            console.log(data); // 打印测试
            socket.send(JSON.stringify(data));
           
        }

        /*摇杆内容*/
        var JoyStick = (function(container, parameters) {
            parameters = parameters || {};
            var title = (typeof parameters.title === "undefined" ? "joystick" : parameters.title),
                width = (typeof parameters.width === "undefined" ? 0 : parameters.width),
                height = (typeof parameters.height === "undefined" ? 0 : parameters.height),
                internalFillColor = (typeof parameters.internalFillColor === "undefined" ? "#00979C" : parameters.internalFillColor),
                internalLineWidth = (typeof parameters.internalLineWidth === "undefined" ? 2 : parameters.internalLineWidth),
                internalStrokeColor = (typeof parameters.internalStrokeColor === "undefined" ? "#00979C" : parameters.internalStrokeColor),
                externalLineWidth = (typeof parameters.externalLineWidth === "undefined" ? 2 : parameters.externalLineWidth),
                externalStrokeColor = (typeof parameters.externalStrokeColor ===  "undefined" ? "#0097BC" : parameters.externalStrokeColor),
                autoReturnToCenter = (typeof parameters.autoReturnToCenter === "undefined" ? true : parameters.autoReturnToCenter);
            
            // Create Canvas element and add it in the Container object
            var objContainer = document.getElementById(container);
            var canvas = document.createElement("canvas");
            canvas.id = title;
            if(width === 0) { width = objContainer.clientWidth; }
            if(height === 0) { height = objContainer.clientHeight; }
            canvas.width = width;
            canvas.height = height;
            objContainer.appendChild(canvas);
            var context = canvas.getContext("2d");
            
            var isPressing = 0;
            var isMoving = 0;
            var isRelease = 0;
            
            var circumference = 2 * Math.PI;
            var internalRadius = (canvas.width - ((canvas.width / 2) + 10)) / 2;
            var maxMoveStick = internalRadius + 5;
            var externalRadius = internalRadius + 30;
            var centerX = canvas.width / 2;
            var centerY = canvas.height / 2;
            var directionHorizontalLimitPos = canvas.width / 10;
            var directionHorizontalLimitNeg = directionHorizontalLimitPos * -1;
            var directionVerticalLimitPos = canvas.height / 10;
            var directionVerticalLimitNeg = directionVerticalLimitPos * -1;
            // Used to save current position of stick
            var movedX = centerX;
            var movedY = centerY;
            
            // Check if the device support the touch or not
            if("ontouchstart" in document.documentElement) {
                canvas.addEventListener("touchstart", onTouchStart, true);
                canvas.addEventListener("touchmove", onTouchMove, true);
                document.addEventListener("touchend", onTouchEnd, true);
            } else {
                canvas.addEventListener("mousedown", onMouseDown, true);
                canvas.addEventListener("mousemove", onMouseMove, true);
                document.addEventListener("mouseup", onMouseUp, true);
            }
            // Draw the object
            drawExternal();
            drawInternal();

            /**
             * @desc Draw the external circle used as reference position
             */
            function drawExternal() {
                context.beginPath();
                context.arc(centerX, centerY, externalRadius, 0, circumference, false);
                context.lineWidth = externalLineWidth;
                context.strokeStyle = externalStrokeColor;
                context.stroke();
            }

            /**
             * @desc Draw the internal stick in the current position the user have moved it
             */
            function drawInternal() {
                context.beginPath();
                if(movedX < internalRadius) { movedX = maxMoveStick; }
                if((movedX + internalRadius) > canvas.width) { movedX = canvas.width - (maxMoveStick); }
                if(movedY < internalRadius) { movedY = maxMoveStick; }
                if((movedY + internalRadius) > canvas.height) { movedY = canvas.height - (maxMoveStick); }
                context.arc(movedX, movedY, internalRadius, 0, circumference, false);
                // create radial gradient
                var grd = context.createRadialGradient(centerX, centerY, 5, centerX, centerY, 200);
                // Light color
                grd.addColorStop(0, internalFillColor);
                // Dark color
                grd.addColorStop(1, internalStrokeColor);
                context.fillStyle = grd;
                context.fill();
                context.lineWidth = internalLineWidth;
                context.strokeStyle = internalStrokeColor;
                context.stroke();
            }

            function postCoordinate(containerId) {
                var xValue = (100 * ((movedX - centerX) / maxMoveStick)).toFixed();
                var yValue = ((100 * ((movedY - centerY) / maxMoveStick)) * -1).toFixed();

                if(containerId === 'joy1Div') {
                    joyX = xValue;
                    joyY = yValue;
                    // 更新 joy1 的显示信息
                    document.getElementById('joy1XValue').textContent = xValue;
                    document.getElementById('joy1YValue').textContent = yValue;
                } else if(containerId === 'joy2Div') {
                    // 限定 x 轴 roll 取值范围
                    let rollValue = parseInt((xValue * 30 / 100).toFixed());
                    if (rollValue < -30) {
                        rollValue = -30;
                    } else if (rollValue > 30) {
                        rollValue = 30;
                    }
                    g_roll = rollValue;

                    // 限定 y 轴 小车高度 取值范围为 15 到 85
                    let heightValue = parseInt(yValue) ;
                    if (heightValue < 15) {
                        heightValue = 15;
                    } else if (heightValue > 85) {
                        heightValue = 85;
                    }
                    g_h = heightValue;

                    // 更新 joy2 的显示信息，显示实际限制后的值
                    document.getElementById('joy2XValue').textContent = g_roll;
                    document.getElementById('joy2YValue').textContent = g_h;
                }

                send_data();
            }

            function releaseControl() {
                joyX = 0;
                joyY = 0;
                g_roll = 0;
                g_h = 32;
                // 重置显示信息
                document.getElementById('joy1XValue').textContent = '0';
                document.getElementById('joy1YValue').textContent = '0';
                document.getElementById('joy2XValue').textContent = '0';
                document.getElementById('joy2YValue').textContent = g_h;
                
                send_data();
            }  

            /**
             * @desc Events for manage touch
             */
            function noTouch(event) {
                isPressing = 0;
                isMoving = 0;
                isRelease = 0;
            }

            function onTouchStart(event) {
                isPressing = 1;
                isMoving = 0;
                isRelease = 0;
            }

            function onTouchMove(event) {
                // Prevent the browser from doing its default thing (scroll, zoom)
                event.preventDefault();
                if(isPressing === 1 && event.targetTouches[0].target === canvas) {
                    isMoving = 1;
                    isRelease = 0;
                    
                    movedX = event.targetTouches[0].pageX;
                    movedY = event.targetTouches[0].pageY;
                    // Manage offset
                    if(canvas.offsetParent.tagName.toUpperCase() === "BODY") {
                        movedX -= canvas.offsetLeft;
                        movedY -= canvas.offsetTop;
                    } else {
                        movedX -= canvas.offsetParent.offsetLeft;
                        movedY -= canvas.offsetParent.offsetTop;
                    }
                    // Delete canvas
                    context.clearRect(0, 0, canvas.width, canvas.height);
                    // Redraw object
                    drawExternal();
                    drawInternal();
                    
                    postCoordinate(canvas.parentNode.id);
                }
            } 

            function onTouchEnd(event) {
                //event.preventDefault();
                isPressing = 0;
                isMoving = 0;
                isRelease = 1;
                
                // If required reset position store variable
                if(autoReturnToCenter) {
                    movedX = centerX;
                    movedY = centerY;
                }
                // Delete canvas
                context.clearRect(0, 0, canvas.width, canvas.height);
                // Redraw object
                drawExternal();
                drawInternal();
                
                releaseControl();
            }

            /**
             * @desc Events for manage mouse
             */
            function noMouse(event) {
                isPressing = 0;
                isMoving = 0;
                isRelease = 0;
            }

            function onMouseDown(event) {
                isPressing = 1;
                isMoving = 0;
                isRelease = 0;
            }

            function onMouseMove(event) {
                if(isPressing === 1) {
                    isMoving = 1;
                    isRelease = 0;
                    
                    movedX = event.pageX;
                    movedY = event.pageY;
                    // Manage offset
                    if(canvas.offsetParent.tagName.toUpperCase() === "BODY") {
                        movedX -= canvas.offsetLeft;
                        movedY -= canvas.offsetTop;
                    } else {
                        movedX -= canvas.offsetParent.offsetLeft;
                        movedY -= canvas.offsetParent.offsetTop;
                    }
                    // Delete canvas
                    context.clearRect(0, 0, canvas.width, canvas.height);
                    // Redraw object
                    drawExternal();
                    drawInternal();
                    
                    postCoordinate(canvas.parentNode.id);
                }
            }

            function onMouseUp(event) {
                isPressing = 0;
                isMoving = 0;
                isRelease = 1;
                
                // If required reset position store variable
                if(autoReturnToCenter) {
                    movedX = centerX;
                    movedY = centerY;
                }
                // Delete canvas
                context.clearRect(0, 0, canvas.width, canvas.height);
                // Redraw object
                drawExternal();
                drawInternal();
                
                releaseControl();
            }

            this.GetX = function () {
                return (100 * ((movedX - centerX) / maxMoveStick)).toFixed();
            };

            /**
             * @desc Normalizzed value of Y move of stick
             * @return Integer from -100 to +100
             */
            this.GetY = function () {
                return ((100 * ((movedY - centerY) / maxMoveStick)) * -1).toFixed();
            };
        });

        var joy1Param = { "title": "1" };  
        var Joy1 = new JoyStick('joy1Div', joy1Param);
        var joy2Param = { "title": "2" };  
        var Joy2 = new JoyStick('joy2Div', joy2Param);
    </script> 
</body>
</html>

)=====";
