/*
 * Name          : joy.js
 * @author       : Roberto D'Amico (Bobboteck)
 * Last modified : 07.01.2020
 * Revision      : 1.1.4
 *
 * Modification History:
 * Date         Version     Modified By		Description
 * 2020-01-07	1.1.4		Roberto D'Amico Close #6 by implementing a new parameter to set the functionality of auto-return to 0 position
 * 2019-11-18	1.1.3		Roberto D'Amico	Close #5 correct indication of East direction
 * 2019-11-12   1.1.2       Roberto D'Amico Removed Fix #4 incorrectly introduced and restored operation with touch devices
 * 2019-11-12   1.1.1       Roberto D'Amico Fixed Issue #4 - Now JoyStick work in any position in the page, not only at 0,0
 * 
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Roberto D'Amico (Bobboteck)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @desc Principal object that draw a joystick, you only need to initialize the object and suggest the HTML container
 * @costructor
 * @param container {String} - HTML object that contains the Joystick
 * @param parameters (optional) - object with following keys:
 *	title {String} (optional) - The ID of canvas (Default value is 'joystick')
 * 	width {Int} (optional) - The width of canvas, if not specified is setted at width of container object (Default value is the width of container object)
 * 	height {Int} (optional) - The height of canvas, if not specified is setted at height of container object (Default value is the height of container object)
 * 	internalFillColor {String} (optional) - Internal color of Stick (Default value is '#00AA00')
 * 	internalLineWidth {Int} (optional) - Border width of Stick (Default value is 2)
 * 	internalStrokeColor {String}(optional) - Border color of Stick (Default value is '#003300')
 * 	externalLineWidth {Int} (optional) - External reference circonference width (Default value is 2)
 * 	externalStrokeColor {String} (optional) - External reference circonference color (Default value is '#008000')
 * 	autoReturnToCenter {Bool} (optional) - Sets the behavior of the stick, whether or not, it should return to zero position when released (Default value is True and return to zero)
 */
var JoyStick = (function (container, parameters) {
    parameters = parameters || {};
    var title = (undefined === parameters.title ? 'joystick' : parameters.title),
        width = (undefined === parameters.width ? 0 : parameters.width),
        height = (undefined === parameters.height ? 0 : parameters.height),
        internalFillColor = (undefined === parameters.internalFillColor ? '#00AA00' : parameters.internalFillColor),
        internalLineWidth = (undefined === parameters.internalLineWidth ? 2 : parameters.internalLineWidth),
        internalStrokeColor = (undefined === parameters.internalStrokeColor ? '#003300' : parameters.internalStrokeColor),
        externalLineWidth = (undefined === parameters.externalLineWidth ? 2 : parameters.externalLineWidth),
        externalStrokeColor = (undefined === parameters.externalStrokeColor ? '#008000' : parameters.externalStrokeColor),
        autoReturnToCenter = (undefined === parameters.autoReturnToCenter ? true : parameters.autoReturnToCenter);

    // Create Canvas element and add it in the Container object
    var objContainer = document.getElementById(container);
    var canvas = document.createElement('canvas');
    canvas.style.borderRadius = "50%";
    canvas.style.overflow = "hidden";
    canvas.id = title;
    if (width == 0) width = objContainer.clientWidth;
    if (height == 0) height = objContainer.clientHeight;
    canvas.width = width;
    canvas.height = height;
    objContainer.appendChild(canvas);
    var context = canvas.getContext('2d');

    var pressed = 0; // Bool - 1=Yes - 0=No
    var circumference = 2 * Math.PI;
    var internalRadius = (canvas.width - ((50 * 2) + 10)) / 2;
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
    if ("ontouchstart" in document.documentElement) {
        canvas.addEventListener('touchstart', onTouchStart, false);
        canvas.addEventListener('touchmove', onTouchMove, false);
        canvas.addEventListener('touchend', onTouchEnd, false);
    }
    else {
        canvas.addEventListener('mousedown', onMouseDown, false);
        canvas.addEventListener('mousemove', onMouseMove, false);
        canvas.addEventListener('mouseup', onMouseUp, false);
    }
    // Draw the object
    drawExternal();
    drawInternal(centerX, centerY);
    /******************************************************
     * Private methods
     *****************************************************/
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
        if (movedX < internalRadius) movedX = maxMoveStick;
        if ((movedX + internalRadius) > canvas.width) movedX = canvas.width - (maxMoveStick);
        if (movedY < internalRadius) movedY = maxMoveStick;
        if ((movedY + internalRadius) > canvas.height) movedY = canvas.height - (maxMoveStick);
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

    /**
     * @desc Events for manage touch
     */
    function onTouchStart(event) {
        pressed = 1;
    }
    function onTouchMove(event) {
        // Prevent the browser from doing its default thing (scroll, zoom)
        event.preventDefault();
        if (pressed == 1) {
            movedX = event.touches[0].pageX;
            movedY = event.touches[0].pageY;
            // Manage offset
            movedX -= canvas.offsetLeft;
            movedY -= canvas.offsetTop;
            // Delete canvas
            context.clearRect(0, 0, canvas.width, canvas.height);
            // Redraw object
            drawExternal();
            drawInternal();
        }
    }
    function onTouchEnd(event) {
        pressed = 0;
        // If required reset position store variable
        if (autoReturnToCenter) {
            movedX = centerX;
            movedY = centerY;
        }
        // Delete canvas
        context.clearRect(0, 0, canvas.width, canvas.height);
        // Redraw object
        drawExternal();
        drawInternal();
        //canvas.unbind('touchmove');
    }
    /**
     * @desc Events for manage mouse
     */
    function onMouseDown(event) {
        pressed = 1;
    }
    function onMouseMove(event) {
        if (pressed == 1) {
            movedX = event.pageX;
            movedY = event.pageY;
            // Manage offset
            movedX -= canvas.offsetLeft;
            movedY -= canvas.offsetTop;
            // Delete canvas
            context.clearRect(0, 0, canvas.width, canvas.height);
            // Redraw object
            drawExternal();
            drawInternal();
        }
    }
    function onMouseUp(event) {
        pressed = 0;
        // If required reset position store variable
        if (autoReturnToCenter) {
            movedX = centerX;
            movedY = centerY;
        }
        // Delete canvas
        context.clearRect(0, 0, canvas.width, canvas.height);
        // Redraw object
        drawExternal();
        drawInternal();
        //canvas.unbind('mousemove');
    }
    /******************************************************
     * Public methods
     *****************************************************/
    /**
     * @desc The width of canvas
     * @return Number of pixel width 
     */
    this.GetWidth = function () {
        return canvas.width;
    };

    /**
     * @desc The height of canvas
     * @return Number of pixel height
     */
    this.GetHeight = function () {
        return canvas.height;
    };

    /**
     * @desc The X position of the cursor relative to the canvas that contains it and to its dimensions
     * @return Number that indicate relative position
     */
    this.GetPosX = function () {
        return movedX;
    };

    /**
     * @desc The Y position of the cursor relative to the canvas that contains it and to its dimensions
     * @return Number that indicate relative position
     */
    this.GetPosY = function () {
        return movedY;
    };

    /**
     * @desc Normalizzed value of X move of stick
     * @return Integer from -100 to +100
     */
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

    /**
     * @desc Get the direction of the cursor as a string that indicates the cardinal points where this is oriented
     * @return String of cardinal point N, NE, E, SE, S, SW, W, NW and C when it is placed in the center
     */
    this.GetDir = function () {
        var result = "";
        var orizontal = movedX - centerX;
        var vertical = movedY - centerY;

        if (vertical >= directionVerticalLimitNeg && vertical <= directionVerticalLimitPos) {
            result = "C";
        }
        if (vertical < directionVerticalLimitNeg) {
            result = "N";
        }
        if (vertical > directionVerticalLimitPos) {
            result = "S";
        }

        if (orizontal < directionHorizontalLimitNeg) {
            if (result == "C") {
                result = "W";
            }
            else {
                result += "W";
            }
        }
        if (orizontal > directionHorizontalLimitPos) {
            if (result == "C") {
                result = "E";
            }
            else {
                result += "E";
            }
        }

        return result;
    };
});
