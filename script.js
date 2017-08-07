/*
 * Websensor video stabilization project
 * https://github.com/jessenie-intel/websensor-video
 *
 * Copyright (c) 2017 Jesse Nieminen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

'use strict';


var nosensors = false;
var accel_sensor = null;
var orientation_sensor = null;
var absori_sensor = null;
var gyroscope = null;
var sensorfreq = 30;
var fps = 30;
var selectedSensor = null;

var constraints = {audio: false,video: {  width: { min: 640, ideal: 640, max: 640 },  height: { min: 480, ideal: 480, max: 480 }, facingMode: { ideal: "environment" }, frameRate: { ideal: fps, max: sensorfreq }}};
var mediaRecorder = null;
var chunks = [];
var videoData = null;
var interval = null;

//var roll = null;
//var pitch = null;
//var yaw = null;
var accel = {"x": null, "y": null, "z": null};
var accel_last = {"x": null, "y": null, "z": null};
var accelNoG = {"x": null, "y": null, "z": null};
var accel_filtered = null;
var aVel = {"x": null, "y": null, "z": null};
var ori = {"roll": null, "pitch": null, "yaw": null, "time": null};
var ori_filtered = null;
var oriInitial = {"roll": null, "pitch": null, "yaw": null, "time": null};
var initialoriobtained = false;
var orientationData = [];       //array to store all the orientation data
var aVelData = [];
var frameData = {"frame": null, "data": null, "time": null, "absori":null, "ori": null, "aVel": null, "accel": null, "accelnog": null, "timeDiff": null};
var dataArray = [];     //array to store all the combined data
var dataArray2 = [];     //array to store all the combined data

var velocity = {"x": 0, "y": 0, "z": 0};

var urlParams = null;

var alpha = 0;
var beta = 0;
var gamma = 0;
var accl = null;

var frame = null;
var timeInitial = null;
var time = null;
var timestamp = null;
var timestamps = [];
var timestampDiffs = [];
var timeAtSensorStart = null;
var nFrame = 0; //frame number with which we can combine timestamp and frame data
var prevFrame = null;      //previous frame
var delay = 0;
var smoothing = 0.85;
var rco = 1;    //determines canvas rotation amount
var sensorframeTimeDiff = 0;    //time difference between sensor and frame data in ms - this is how much the timestamps differ
var k = 10;     //bigger window size
var l = 2;      //smaller window size
var h = 20;     //alarm parameter for splitting using gyro data, chosen manually
var mu = 0;     //small drift term used in LS filter, chosen manually


//canvas
var canvas = document.getElementById('myCanvas');
var canvas2 = document.getElementById('myCanvas2');
var ctx = canvas.getContext('2d');
var ctx2 = canvas2.getContext('2d');

//video element
var videoElement = document.querySelector('video');
videoElement.controls = false;
var videoURLBase = null;
var duration = null;

var ref = null;
var extraFrames = 0;
var x = 0;
var y = 0;      //position for the square
var angle = 0;

var cameraPath = [];    //smoothed camera path
var cameraPath2 = [];    //array of canvas coordinates describing the camera path
var cameraCoord = {"x": null, "y": null, "time": null};

var absori = {"ori": null, "time": null};
var absoris = [];

var tempArray2 = [];
var anglesArray2 = [];
var filteredAnglesArray = [];

var recordingStarted = false;

//Sliders
var slider_delay = document.getElementById("slider_delay");
var slider_delay_div = document.getElementById("slider_delay_amount");
slider_delay.onchange = () => {
        delay = slider_delay.value;
        slider_delay_div.innerHTML = delay;
        console.log("Delay:", delay);
};

var slider_smoothing = document.getElementById("slider_smoothing");
var slider_smoothing_div = document.getElementById("slider_smoothing_amount");
slider_smoothing.onchange = () => {
        smoothing = slider_smoothing.value;
        slider_smoothing_div.innerHTML = smoothing;
        console.log("Smoothing:", smoothing);
};

/*var slider_rco = document.getElementById("slider_delay");
var slider_rco_div = document.getElementById("slider_delay_amount");
slider_rco.onchange = () => {
        rco = slider_rco.value;
        slider_rco_div.innerHTML = rco;
        console.log("Rotation coefficient:", rco);
};*/

class HighPassFilterData {      //https://w3c.github.io/motion-sensors/#pass-filters
  constructor(reading, cutoffFrequency) {
    Object.assign(this, { x: reading.x, y: reading.y, z: reading.z });
    this.cutoff = cutoffFrequency;
    this.timestamp = reading.timestamp;
  }

  update(reading) {
    let dt = reading.timestamp - this.timestamp / 1000;
    this.timestamp = reading.timestamp;

    for (let i of ["x", "y", "z"]) {
      let alpha = this.cutoff / (this.cutoff + dt);
      this[i] = this[i] + alpha * (reading[i] - this[i]);
    }
  }
};


class LowPassFilterData {       //https://w3c.github.io/motion-sensors/#pass-filters
  constructor(reading, bias) {
    Object.assign(this, { x: reading.x, y: reading.y, z: reading.z });
    this.bias = bias;
  }
        update(reading) {
                this.x = this.x * this.bias + reading.x * (1 - this.bias);
                this.y = this.y * this.bias + reading.y * (1 - this.bias);
                this.z = this.z * this.bias + reading.z * (1 - this.bias);
        }
};

class LowPassFilterOrientation {
  constructor(reading, bias) {
    Object.assign(this, { roll: reading.roll, pitch: reading.pitch, yaw: reading.yaw });
    this.bias = bias;
  }
        update(reading) {
                this.roll = this.roll * this.bias + reading.roll * (1 - this.bias);
                this.pitch = this.pitch * this.bias + reading.pitch * (1 - this.bias);
                this.yaw = this.yaw * this.bias + reading.yaw * (1 - this.bias);
        }
};

class KalmanFilter {
  constructor() {
    this.Q_angle = 0.01;
    this.Q_gyro = 0.0003;
    this.R_angle = 0.01;

    this.reset();
  }

  reset() {
    this.angle = 0.0;
    this.bias = 0;

    this.P00 = 0;
    this.P01 = 0;
    this.P10 = 0;
    this.P11 = 0;
  }

  filter(accAngle, gyroRate, dt) {
    this.angle += dt * (gyroRate - this.bias);

    this.P00 += -dt * (this.P10 + this.P01) + this.Q_angle * dt;
    this.P01 += -dt * this.P11;
    this.P10 += -dt * this.P11;
    this.P11 += + this.Q_gyro * dt;

    let axis = accAngle - this.angle;
    let S = this.P00 + this.R_angle;
    let K0 = this.P00 / S;
    let K1 = this.P10 / S;

    this.angle += K0 * axis;
    this.bias  += K1 * axis;

    this.P00 -= K0 * this.P00;
    this.P01 -= K0 * this.P01;
    this.P10 -= K1 * this.P00;
    this.P11 -= K1 * this.P01;

    return this.angle;
  }
};

class OriSensor {
        constructor() {
        const sensor = new RelativeOrientationSensor({ frequency: sensorfreq });
        const mat4 = new Float32Array(16);
        const euler = new Float32Array(3);
        sensor.onreading = () => {
                sensor.populateMatrix(mat4);
                toEulerianAngle(sensor.quaternion, euler);      //From quaternion to Euler angles
                this.roll = euler[0];
                this.pitch = euler[1];
                this.yaw = euler[2];
                this.timestamp = sensor.timestamp;
                if (this.onreading) this.onreading();
        };
        sensor.onactivate = () => {
                if (this.onactivate) this.onactivate();
        };
        const start = () => sensor.start();
        Object.assign(this, { start });
        }
}

function magnitude(vector)      //Calculate the magnitude of a vector
{
return Math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

/*
*       Input: Gyroscope data (angular velocity)
*       Output: Array of indices indicating where to split the Hann windowing
*/
function getHannIndices(aVelData)       //Splits the Hann windowing into parts, where the change (angular velocity) is fast the windows are smaller (indices closer together) and where it is slow the windows are larger TODO: No drift term yet, incorrect algorithm implemented
{
        let indices = [];
        indices.push(0);        //the first index is always 0
        let gyroDataArray = [];
        aVelData.forEach(function(entry) {
        let gyroData = (({ x, y, z }) => ({ x, y, z }))(entry);    //only select x,y,z
        gyroDataArray.push(gyroData);
        });
        let testStatistic1Sum = 0;   //Sum to track the amount of movement that has happened since the last index
        let testStatistic2Sum = 0;   //Sum to track the amount of movement that has happened since the last index
        let magnitudesUntilNow = [];
        for(let i=1; i<gyroDataArray.length; i++)
        {
                let distanceMeasure1 = 0;
                let distanceMeasure2 = 0;
                let testStatistic1 = 0;
                let testStatistic2 = 0;
                let magnitude = Math.sqrt(gyroDataArray[i].x * gyroDataArray[i].x + gyroDataArray[i].y * gyroDataArray[i].y + gyroDataArray[i].z * gyroDataArray[i].z);
                magnitudesUntilNow.push(magnitude);
                let lsEstimation = magnitudesUntilNow => magnitudesUntilNow.reduce( ( p, c ) => p + c, 0 ) / magnitudesUntilNow.length; //average of magnitudes until now
                if(i > 0)
                {
                        let magnitudePrev = Math.sqrt(gyroDataArray[i-1].x * gyroDataArray[i-1].x + gyroDataArray[i-1].y * gyroDataArray[i-1].y + gyroDataArray[i-1].z * gyroDataArray[i-1].z);
                        distanceMeasure1 = magnitude - magnitudePrev;
                        distanceMeasure2 = -magnitude + magnitudePrev;
                }
                else
                {
                        distanceMeasure1 = magnitude - lsEstimation;
                        distanceMeasure2 = -magnitude + lsEstimation;
                }
                if(i > 0)
                {
                        let testStatistic1Prev = testStatistic1;    //store previous value
                        let testStatistic2Prev = testStatistic2;    //store previous value
                        testStatistic1 = Math.max(testStatistic1Prev + distanceMeasure1 - mu, 0);
                        testStatistic2 = Math.max(testStatistic2Prev + distanceMeasure2 - mu, 0);
                }
                else
                {
                        testStatistic1 = Math.max(distanceMeasure1 - mu, 0);
                        testStatistic2 = Math.max(distanceMeasure2 - mu, 0);
                }
                testStatistic1Sum = testStatistic1Sum + testStatistic1;
                testStatistic2Sum = testStatistic2Sum + testStatistic2;
                console.log(testStatistic1Sum, testStatistic1Sum); 
                if(testStatistic1Sum > 5 && i > 0)   //Cannot push 0 twice!
                {
                        indices.push(i);
                        //lsEstimation = 0;
                        testStatistic1Sum = 0;
                        testStatistic2Sum = 0;
                        magnitudesUntilNow = [];
                }
        }
        if(indices[indices.length-1] !== gyroDataArray.length-1)
        { 
                indices.push(gyroDataArray.length-1);   //last index is always the index of the last element
        }
        return indices;
}

function hannWindow(dataIn, indices) {   //Low-pass filter with Hann window of length dataIn.length TODO: Should split into smaller lengths as indicated by indices
        //Split using the indices
        let dataInByIndices = [];
        for(let i=0; i<indices.length-1; i++)
        {
                let dataInPart = dataIn.slice(indices[i], indices[i+1]);
                dataInByIndices.push(dataInPart);
        }
        console.log(dataInByIndices);
        let dataOut = [];
        for (let partIndex = 0; partIndex < dataInByIndices.length; partIndex++)
        {
                let part = dataInByIndices[partIndex];
                //console.log(part);
                for (let i = 0; i < part.length; i++) {
                        let multiplier = 0.5 * (1 - Math.cos(2*Math.PI*i/(part.length-1))); //the weight
                        let value = part[i].map(function(x) { return x * multiplier; });;
                        var b = new Float32Array(16);     //need to push by value
                        Object.assign(b, value);
                        dataOut.push(b);
                }
        }
        return dataOut;   
}

/*
*       Input: Indices according to which the segments are split
*       Output: Intervals of the respective segments
*/
function findSegments(indices) //Splits the segments into two types: segments for which the length is greater than the wider window length k (big segments) and the rest (small segments) by indices
{
        //Arrays of intervals
        let bigSegments = [];
        let smallSegments = [];
        for(let i=1; i<indices.length; i++)
        {
                let segmentLength = indices[i]-indices[i-1];
                let interval = [];
                i > 1 ? interval = [indices[i-1]+1, indices[i]] : interval = [indices[i-1], indices[i]];
                segmentLength > k ? bigSegments.push(interval) : smallSegments.push(interval);
        }
        return {"bigSegments": bigSegments, "smallSegments": smallSegments};
}

function doWMA( array, weightedPeriod ) {       //https://www.reddit.com/r/learnprogramming/comments/39cg7r/javascript_looking_for_sample_weighted_moving/cs3e08f/
    var weightedArray = [];
    for( var i = 0; i <= array.length - weightedPeriod; i++ ) {
        var sum = 0;
        for( var j = 0; j < weightedPeriod; j++ ) {
            sum += array[ i + j ] * ( weightedPeriod - j );
        }
        weightedArray[i] = sum / (( weightedPeriod * ( weightedPeriod + 1 )) / 2 );
    }
    return weightedArray;
}

function smooth(values, alpha) {        //https://stackoverflow.com/q/32788836
    var weighted = average(values) * alpha;
    var smoothed = [];
    for (var i in values) {
        var curr = values[i];
        var prev = smoothed[i - 1] || values[values.length - 1];
        var next = curr || values[0];
        var improved = Number(average([weighted, prev, curr, next]).toFixed(2));
        smoothed.push(improved);
    }
    return smoothed;
}

function average(data) {
    var sum = data.reduce(function(sum, value) {
        return sum + value;
    }, 0);
    var avg = sum / data.length;
    return avg;
}


/*
* Input: Array of orientation data including timestamps
* Output: Euler angles
*/
function lpFilterOri(quatArrayIn, gyroData)
{
        let quatArray = [];     //array of quaternions without timestamps
        let tempArray = [];     //for storing quatArray orientation values
        quatArray = quatArrayIn;
        let quatArray2 = [];        //windowed quaternions
        for (let i=0; i<quatArray.length; i++)
        {
                let quat = [];
                Object.entries(quatArray[i]).forEach(
                    ([key, value]) => quat.push(value)
                );
                quatArray2.push(quat[0]);
        }
        console.log(quatArray2);
        let indices = getHannIndices(gyroData);
        let segments = findSegments(indices);
        let quatArrayFiltered = hannWindow(quatArray2, indices); //Hann window to low-pass filter
        console.log(quatArrayFiltered);
        var anglesArray = [];   //Euler angles corresponding to the low-pass filtered quaternions
        for (let i=0; i<quatArrayFiltered.length; i++)
                {
                const euler = new Float32Array(3);
                toEulerianAngle(quatArrayFiltered[i], euler);      //From quaternion to Euler angles
                let roll = euler[0];
                let pitch = euler[1];
                let yaw = euler[2];
                let angles = {"roll": roll, "pitch": pitch, "yaw": yaw, "time": null};
                var b = new Object;     //need to push by value
                Object.assign(b, angles);
                anglesArray.push(angles);
        }
        return anglesArray;
}

function update_debug()
{
                        document.getElementById("ori").textContent = `Orientation: ${ori.roll} ${ori.pitch} ${ori.yaw}`;
                        document.getElementById("accel").textContent = `Acceleration: ${accel.x} ${accel.y} ${accel.z}, M ${magnitude(accel).toFixed(3)}`;
                        document.getElementById("accelnog").textContent = `Linear acceleration (no gravity): ${accelNoG.x} ${accelNoG.y} ${accelNoG.z}, M ${magnitude(accelNoG).toFixed(3)}`;
                        document.getElementById("rrate").textContent = `Rotation rate: ${aVel.x} ${aVel.y} ${aVel.z}, M ${magnitude(aVel).toFixed(3)} and alpha ${alpha} beta ${beta} gamma ${gamma}`;
                        document.getElementById("selectedSensor").textContent = `${selectedSensor}`;
}

function exportData() //https://stackoverflow.com/a/13405322
{
        var exportData = JSON.stringify(dataArray);
        window.open('data:text/csv;charset=utf-8,' + escape(exportData));
}

function selectSensor() {
        selectedSensor = document.getElementById("selectSensor").value;
        console.log(selectedSensor, "selected");
}

function buildCameraPath(dataArray) {    //Build the shaky camera path from the sensor measurements (convert to canvas coordinates) using projection TODO: Change to using the low-pass filtered angles
        let cameraPath = [];
        for (let i=0; i<dataArray.length; i++)
        {
                let ori = dataArray[i].ori;
                let oriDiff = null;
                if(ori !== undefined)
                {
                        oriDiff = {"roll": ori.roll-oriInitial.roll, "pitch": ori.pitch-oriInitial.pitch, "yaw": ori.yaw-oriInitial.yaw};
                        //console.log(oriDiff.yaw);
                        cameraCoord.x = (2/2)* Math.sin(oriDiff.yaw*2) * canvas.width;
                        cameraCoord.y = (2/2)* Math.sin(oriDiff.roll*2) * canvas.height;
                        var b = new Object;     //need to push by value
                        Object.assign(b, cameraCoord);
                        cameraPath.push(b);
                }
        }
        return cameraPath;
}

/*
*       Input: The low-pass filtered angles
*       Output: The path of the camera in terms of canvas coordinates x,y
*/
function buildCameraPath2(angleArray) {    //Build the stabilized camera path from the sensor measurements (convert to canvas coordinates) using projection
        let cameraPath = [];
        for (let i=0; i<angleArray.length; i++)
        {
                let ori = angleArray[i];
                let oriDiff = null;
                if(ori !== undefined)
                {
                        oriDiff = {"roll": ori.roll-oriInitial.roll, "pitch": ori.pitch-oriInitial.pitch, "yaw": ori.yaw-oriInitial.yaw};
                        //console.log(oriDiff.yaw);
                        cameraCoord.x = (2/2)* Math.sin(oriDiff.yaw*2) * canvas.width;  //These coefficients chosen by testing, still need to improve
                        cameraCoord.y = (2/2)* Math.sin(oriDiff.roll*2) * canvas.height;
                        var b = new Object;     //need to push by value
                        Object.assign(b, cameraCoord);
                        cameraPath.push(b);
                }
        }
        return cameraPath;
}

//WINDOWS 10 HAS DIFFERENT CONVENTION: Yaw z, pitch x, roll y
function toEulerianAngle(quat, out)
{
        const ysqr = quat[1] ** 2;

        // Roll (x-axis rotation).
        const t0 = 2 * (quat[3] * quat[0] + quat[1] * quat[2]);
        const t1 = 1 - 2 * (ysqr + quat[0] ** 2);
        out[0] = Math.atan2(t0, t1);
        // Pitch (y-axis rotation).
        let t2 = 2 * (quat[3] * quat[1] - quat[2] * quat[0]);
        t2 = t2 > 1 ? 1 : t2;
        t2 = t2 < -1 ? -1 : t2;
        out[1] = Math.asin(t2);
        // Yaw (z-axis rotation).
        const t3 = 2 * (quat[3] * quat[2] + quat[0] * quat[1]);
        const t4 = 1 - 2 * (ysqr + quat[2] ** 2);
        out[2] = Math.atan2(t3, t4);
        return out;
}

function errorCallback(error){
	console.log("error: ", error);	
}

function startDemo () {
		navigator.getUserMedia(constraints, startRecording, errorCallback);
}

function startSensors() {
                try {
                //timeInitial = Date.now();
                timeInitial = window.performance.now();
                const bias = 0.98;
                //Initialize sensors
                accel_sensor = new Accelerometer({frequency: sensorfreq});
                // Remove drift with a high pass filter.
                const accel_filtered =  new HighPassFilterData(accel_sensor, 0.8);
                accel_sensor.onreading = () => {
                        accel = {"x": accel_sensor.x, "y": accel_sensor.y, "z": accel_sensor.z, "time": accel_sensor.timestamp};
                        //accel = {"x": (1/2)*(accel_last.x + accel_sensor.x), "y": (1/2)*(accel_last.y + accel_sensor.y), "z": (1/2)*(accel_last.z + accel_sensor.z)};
                        //accel_last = accel;     //for smoothing the data
                        //accel_filtered.update(accel_sensor);                   
                        //accel = accel_filtered;                        
                        //accelNoG = {x:accel.x - gravity.x, y:accel.y - gravity.y, z:accel.z - gravity.z};
                };
                accel_sensor.onactivate = () => {
                };
                accel_sensor.start();
                gyroscope = new Gyroscope({frequency: sensorfreq});
                //accl = new Accelerometer({frequency: sensorfreq});
                const gyro_filtered = new LowPassFilterData(gyroscope, 0.85);
                gyroscope.onreading = () => {
                        gyro_filtered.update(gyroscope);
                        //aVel = {x:gyro_data.x, y:gyro_data.y, z:gyro_data.z};
                        //aVel = {x:gyroscope.x, y:gyroscope.y, z:gyroscope.z, alpha: alpha};
                        //Determine orientation with accelerometer and gyroscope. Below from https://w3c.github.io/motion-sensors/#complementary-filters
                        let dt = timestamp ? (gyroscope.timestamp - timestamp) / 1000 : 0;
                        timestamp = gyroscope.timestamp;

                        // Treat the acceleration vector as an orientation vector by normalizing it.
                        // Keep in mind that the if the device is flipped, the vector will just be
                        // pointing in the other direction, so we have no way to know from the
                        // accelerometer data which way the device is oriented.
                        const norm = Math.sqrt(accel_sensor.x ** 2 + accel_sensor.y ** 2 + accel_sensor.z ** 2);

                        // As we only can cover half (PI rad) of the full spectrum (2*PI rad) we multiply
                        // the unit vector with values from [-1, 1] with PI/2, covering [-PI/2, PI/2].
                        const scale = Math.PI / 2;

                        const zeroBias = 0.02;
                        
                        //console.log(accel_sensor.timestamp, gyroscope.timestamp, accel_sensor.timestamp - gyroscope.timestamp);
                        //alpha = (1 - zeroBias) * (alpha + gyroscope.z * dt);                        
                        alpha = alpha + gyro_filtered.z * dt;
                        beta = bias * (beta + gyroscope.x * dt) + (1.0 - bias) * (accel_sensor.x * scale / norm);
                        gamma = bias * (gamma + gyro_filtered.y * dt) + (1.0 - bias) * (accel_sensor.y * -scale / norm);
//gamma = (gamma + gyro_filtered.y * dt);
                        aVel = {x:gyroscope.x, y:gyroscope.y, z:gyroscope.z, alpha: alpha, beta: beta, gamma: gamma};
//console.log(Date.now());
                };
                gyroscope.onactivate = () => {
                };
                gyroscope.start();
                orientation_sensor = new OriSensor({frequency: sensorfreq});
                //Low-pass filter the orientation data. Cannot initialize normally because undefined
                const ori_filtered =  new LowPassFilterOrientation({"roll":null, "pitch":null, "yaw":null}, 0.9);
                //console.log(ori_filtered);
                orientation_sensor.onreading = () => {
                        let roll = orientation_sensor.roll;
                        let pitch = orientation_sensor.pitch;
                        let yaw = orientation_sensor.yaw;
                        let time_sensor = orientation_sensor.timestamp;
                        let time_window = window.performance.now();
                        if(!initialoriobtained && recordingStarted) //obtain initial orientation
                        {
                                oriInitial = {"roll": orientation_sensor.roll, "pitch": orientation_sensor.pitch, "yaw": orientation_sensor.yaw, "time": orientation_sensor.timestamp};
                                //timeAtSensorStart = orientation_sensor.timestamp;
                                timeAtSensorStart = window.performance.now();
                                initialoriobtained = true;
                                sensorframeTimeDiff = timeAtSensorStart - timeInitial;
                                console.log("Initial orientation obtained");
                        }
                        ori = {"roll": orientation_sensor.roll, "pitch": orientation_sensor.pitch, "yaw": orientation_sensor.yaw, "time_sensor": time_sensor, "time_window": time_window};
                        ori_filtered.update(ori);
                };
                orientation_sensor.onactivate = () => {
                };
                orientation_sensor.start();
                absori_sensor = new AbsoluteOrientationSensor({frequency: sensorfreq});
                const mat4 = new Float32Array(16);
                const euler = new Float32Array(3);
                absori_sensor.onreading = () => {
                        absori_sensor.populateMatrix(mat4);
                        absori = {"ori": mat4, "time": absori_sensor.timestamp};     //need to push by value
                
                        //Object.assign(b.ori, mat4);
                        //absoris.push(b);
                };
                absori_sensor.onactivate = () => {
                };
                absori_sensor.start();
                }
                catch(err) {
                        console.log(err.message);
                        console.log("Your browser doesn't seem to support generic sensors. If you are running Chrome, please enable it in about:flags.");
                        //this.innerHTML = "Your browser doesn't seem to support generic sensors. If you are running Chrome, please enable it in about:flags";
                        nosensors = true;
                }
}

document.addEventListener('DOMContentLoaded', function() {
    //Read URL params
    urlParams = new URLSearchParams(window.location.search);
    nosensors = urlParams.has('nosensors'); //to specify whether or not to use sensors in the URL
    console.log("Sensors (URL):", !nosensors);
    startSensors();     //start sensors instantly to avoid gyro drift
}, false);

function startRecording(stream) {
                recordingStarted = true;
                interval=window.setInterval(update_debug,100);
		mediaRecorder = new MediaRecorder(stream);

	        mediaRecorder.start(1000/(2*fps));  //argument blob length in ms
                frame = 0;

	        var url = window.URL;
	        videoElement.src = url ? url.createObjectURL(stream) : stream;	        
                //videoElement.play();

/*      Event listeners below   */
mediaRecorder.ondataavailable = function(e) {
        //console.log("Data available", e);
        //console.log(time);
        frameData.frame = frame;
        time = Date.now();
        timestamps.push(time);
        frameData.time = time;
        timestampDiffs.push(time-timeAtSensorStart);
        chunks.push(e.data);
        frameData.data = e.data; 
        absoris.push(absori);        
        orientationData.push(ori);
        aVelData.push(aVel);
        frameData.absori = absori;
        frameData.ori = ori;
        frameData.aVel = aVel;
        frameData.accel = accel;        //maybe should use filtered acceleration instead?
        frameData.timeDiff = time-timeAtSensorStart;
        var b = new Object;     //need to push by value
        Object.assign(b, frameData);
        dataArray.push(b);
        frameData = {"frame": null, "data": null, "time": null, "absori":null, "ori": null, "aVel": null, "accel": null, "accelnog": null, "timeDiff": null};
        frame = frame + 1;
};

mediaRecorder.onerror = function(e){
        console.log('Error: ', e);
};


mediaRecorder.onstart = function(){
        console.log("Recording started", mediaRecorder.state);
};

mediaRecorder.onstop = function(){
        var blob = new Blob(chunks, {type: "video/webm"});
        chunks = [];

        videoURLBase = window.URL.createObjectURL(blob);

        videoElement.src = videoURLBase + "#xywh=pixel:0,0,320,240";
        videoElement.load();
                        
                        //resize canvas
videoElement.addEventListener('loadedmetadata', function() {
        console.log("Loaded metadata");
        canvas.width = videoElement.videoWidth;
        canvas.height = videoElement.videoHeight;
        canvas2.width = videoElement.videoWidth;
        canvas2.height = videoElement.videoHeight;
        ctx.save();     //save canvas state for later restoration
});
        duration = (dataArray[dataArray.length-1].time - dataArray[0].time)/1000;      //duration in s

videoElement.addEventListener('play', function() { 
        videoElement.play();
        nFrame = 0;
        let durationPerFrame = duration*1000/dataArray.length;   //frame duration in ms
        let tempCameraPath = {"x": null, "y": null};
        //Hanning window, first process data
        filteredAnglesArray = lpFilterOri(absoris, aVelData);       
        //console.log(filteredAnglesArray);
        cameraPath = buildCameraPath(dataArray);     //build camera path
        let stableCameraPath = buildCameraPath2(filteredAnglesArray);
        //console.log(cameraPath);
        //console.log(stableCameraPath);
        let weightedIndex = 10;
        tempCameraPath.x = doWMA(cameraPath.map(a => a.x), weightedIndex);       //smoothen the path
        tempCameraPath.y = doWMA(cameraPath.map(a => a.y), weightedIndex);      //smoothen the path
        //need to add first elements separately due to MA filter losing them. Note: they will of course not be filtered
        for(let i=0; i<tempCameraPath.x.length+weightedIndex-1; i++)
        {
                if(i<weightedIndex)
                {
                        let xy = {"x": cameraPath[i].x, "y": cameraPath[i].y};
                        cameraPath2.push(xy);
                }
                else
                {
                        let xy = {"x": tempCameraPath.x[i], "y": tempCameraPath.y[i]};
                        cameraPath2.push(xy);
                }
        }        
        console.log(cameraPath2);
        cameraPath2 = stableCameraPath;     //comment for smoothing
        readFrameData();    //reads the video into dataArray2
}, false);
	        };

	        mediaRecorder.onpause = function(){
	        }

	        mediaRecorder.onresume = function(){
	        }

	        mediaRecorder.onwarning = function(e){
	        };
}

function stopRecording(){
	mediaRecorder.stop();
        //Now stabilize
	videoElement.controls = true;
}
//Idea: copy video to canvas, operate on the video, and then use the canvas with the stabilized video as source for the video element
function readFrameData() {     //Read video data from blob to object form with pixel data we can operate on
        //TODO: sensor readings and frame data in desync - frame data too late/sensor data ahead
        nFrame = Math.floor(videoElement.currentTime*fps);
        let dx = 0;
        let dy = 0;
        let trans = {"x": null, "y": null};
        //let delay = -10;
        var timeFromStart = null;
        let frameDataL = (nFrame-delay >=0 && nFrame-delay <= dataArray.length) ? dataArray[nFrame - delay] : dataArray[nFrame];
        if(nFrame === 0 && !videoElement.ended)
        {
                //console.log(dataArray);
                //timeAtSensorStart = frameDataL.ori.time;
        }
        else if(nFrame !== 0 && nFrame !== prevFrame && frameDataL !== undefined && nFrame < videoElement.duration * fps)    //all subsequent frames
        {
                let frame = frameDataL.frame;
                //console.log(frame, nFrame);
                //console.log(nFrame);
                //console.log(dataL);
                timeFromStart = frameDataL.time - timeAtSensorStart; //time since recording start (in ms)
                //console.log(timeFromStart);
                //console.log(frameDataL);
                //console.log(nFrame);
                //console.log(videoElement.webkitDecodedFrameCount);      //only works in webkit browsers
                //console.log(timestamps[nFrame] - timestamps[0], videoElement.currentTime);
                //while(!videoElement.ended)
                //{
                //if(cameraPath !== undefined)
                //{
                //}                
                let oriDiff = null;
                let deltaT = frameDataL.time - dataArray[nFrame-1].time;
                //console.log(deltaT);
                let acceleration_filtered = null;
                //if(magnitude(frameDataL.accel) > 0.5)    //filter out small values in acceleration (noise)
                //{
                        acceleration_filtered = frameDataL.accel;
                /*}
                else
                {
                        acceleration_filtered = {"x":0, "y":0, "z": 0};
                }*/
                velocity = {"x": velocity.x + acceleration_filtered.x * deltaT/1000, "y": velocity.y + acceleration_filtered.y * deltaT/1000, "z": velocity.z + acceleration_filtered.z * deltaT/1000};    //velocity per second TODO: add friction
                //console.log(velocity);
                /*if(dataL === undefined)
                {
                        var dataL = new Object;     //need to push by value
                        Object.assign(dataL, dataArray);
                }*/
                //console.log(dataL);
                        //videoElement.playbackRate = 0.5;        //fix playback being too fast
                        //let ori = orientationData[nFrame];
                        let ori = frameDataL.ori;
                        //ori = dataArrayL[nFrame].ori;
                        let aVel = frameDataL.aVel;
                        //console.log(nFrame, ori, aVel);
                        if(ori !== undefined)
                        {
                                oriDiff = {"roll": ori.roll-oriInitial.roll, "pitch": ori.pitch-oriInitial.pitch, "yaw": ori.yaw-oriInitial.yaw};
                       }
                        //accelerometer not taken into account atm
                                //dx = -(videoElement.videoWidth*(aVel.y/(2)) + 0*videoElement.videoWidth*velocity.x)*deltaT/1000;
                                //dy = (-videoElement.videoHeight*(aVel.x/(2)) + 0*videoElement.videoHeight*velocity.y)*deltaT/1000;
                                //x = x + dx;
                                //y = y + dy;
                                //x = 100*oriDiff.yaw;
                                //y = 100*oriDiff.roll;
                                //x = videoElement.videoWidth*(oriDiff.yaw/(Math.PI));
                                //y = -videoElement.videoHeight*(oriDiff.roll/(Math.PI));     //each 2pi means 1 video height
                                //angle = {"alpha":oriDiff.roll, "beta":oriDiff.pitch, "gamma":oriDiff.yaw};  //angle from ori sensor
                                angle = {"alpha":frameDataL.aVel.alpha - dataArray[0].aVel.alpha, "beta":frameDataL.aVel.beta, "gamma":frameDataL.aVel.gamma - dataArray[0].aVel.gamma};        //angle from gyro
                                //console.log(x, y, angle);
        //Modifying canvas size, we can show only the desired part of the video TODO: modify according to stabilization box
        //canvas.width = videoElement.videoWidth;
        //canvas.height = videoElement.videoHeight;
                        //let cTime = (nFrame/dataArray.length);
                        //console.log(cTime, duration);
                        //videoElement.pause();
                        //videoElement.currentTime = timeFromStart/1000;
    /*var timer = setInterval(function() {
        if (videoElement.readyState ==4 || !videoElement.paused) {
            videoElement.play();
        //render video and rect
        let widthR = 0.8*canvas.width;
        let heightR = 0.8*canvas.height;
        //let videoURL = videoURLBase + "#xywh=pixel:0,0,320,240";
        //videoElement.currentTime = timeFromStart/1000;        //TODO: fix currentTime
        //videoElement.currentTime = (nFrame/dataArray.length)*videoElement.duration;
        //videoElement.currentTime = parseFloat(videoElement.duration * Math.random().toFixed(3));
        //videoElement.src = videoURL;
        //videoElement.load();
        //videoElement.play();
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.drawImage(videoElement,0,0, videoElement.videoWidth, videoElement.videoHeight);
        ctx.beginPath();
        ctx.rect(x+0.1*canvas.width,y+0.1*canvas.height,widthR,heightR);
        ctx.stroke();
            clearInterval(timer);
        }       
    }, 50);*/
                        //console.log(videoElement.currentTime);                        
                               //}          
                        //console.log(x, y);

                        //ctx.drawImage(videoElement, 0, 0);
                        //let imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
                        /*let pixeldataArray = [];
                        //loop through every pixel
                        for(let i=0; i<imageData.data.length; i = i+4)
                        {
                                let pixeldata = {"red": imageData.data[i], "green":imageData.data[i+1], "blue": imageData.data[i+2], "alpha":imageData.data[i+3]};
                                //pixeldataArray.push(pixeldata);
                        }
                        //pixeldataArray.push(imageData);*/
                        //if(ori !== undefined) {
                                //let timestamp = dataArray[nFrame].time;
                                //let frameData2 = {"imagedata": imageData, "time": timestamp, "oridiff": oriDiff};
                                //dataArray2.push(frameData2);
                                //console.log(pixeldataArray);
                                //newImageData.data = data;
                            // Draw the pixels onto the visible canvas
                            //ctx.putImageData(newImageData,0,0);
                                //ctx.putImageData(imageData, 0, 0)
                                //xD
                        //}
                //} 
                /*
                if(dataArray2.length === timestamps.length)     //now we have read the whole blob - should use callback here instead of if condition
                {
                        console.log("ended");
                        cancelAnimationFrame(ref);
                        console.log(dataArray2);
                        stabilize(dataArray2);
                } */ 
        //render video and rect
        let widthR = 0.6*videoElement.videoWidth;
        let heightR = 0.6*videoElement.videoHeight;
        dx = -cameraPath2[nFrame].x;
        dy = -cameraPath2[nFrame].y;
        trans = {"x": videoElement.videoWidth/2 + dx, "y": videoElement.videoHeight/2 + dy};
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.save();

        //NOTE: Direction of translations and rotations depend on camera used (front or back)
        ctx.translate(trans.x, trans.y);
        ctx.rotate(-angle.alpha);       //negative if rear camera, positive if front camera
        ctx.drawImage(videoElement,-videoElement.videoWidth/2,-videoElement.videoHeight/2, videoElement.videoWidth, videoElement.videoHeight);      
        ctx.rotate(angle.alpha); //positive if rear camera, negative if front camera
        ctx.translate(-trans.x, -trans.y);
        ctx.restore();
        ctx.beginPath();
        ctx.rect((canvas.width-widthR)/2,(canvas.height-heightR)/2,widthR,heightR);
        var imgData=ctx.getImageData((canvas.width-widthR)/2,(canvas.height-heightR)/2,widthR,heightR);
        ctx.stroke();
        //ctx2.scale(1/0.8, 1/0.8);
        //ctx2.translate(-videoElement.videoWidth/2 - videoElement.videoWidth * (1.5*angle.gamma/(Math.PI)), 0);
        ctx2.putImageData(imgData, 0, 0, 0, 0, canvas2.width, canvas2.height);
        }
        if(videoElement.ended || nFrame >= videoElement.duration * fps)  //video ended
        {
                console.log("ended");
                cancelAnimationFrame(ref);
                console.log(cameraPath2);
                if(cameraPath2 !== undefined) {
                //console.log(cameraPath);
                for(let i=0; i<nFrame; i++)
                {
                        ctx.fillRect(videoElement.videoWidth/2 + cameraPath2[i].x,videoElement.videoHeight/2 + cameraPath2[i].y,3,3);
                        //console.log(cameraPath2[i].x, cameraPath2[i].y);
                }
        //console.log(cameraPath.length, nFrame);
                }
        }
        /*else if(nFrame >= orientationData.length-1)
        {
                //x = 0;
                //y = 0;
                extraFrames = extraFrames + nFrame;
                prevFrame = null;
                nFrame = 0;
                cancelAnimationFrame(ref);
        }*/
        else
        {
                prevFrame = nFrame;
                //nFrame = nFrame + 1;
                ref = requestAnimationFrame(readFrameData);
        }
}

function stabilize(dataArrayArg) { //Create a stabilized video from the pixel data given as input
        let frame = dataArrayArg[0];      //first frame
        console.log(frame);
        //ctx.drawImage(frame.imagedata,0,0, videoElement.videoWidth, videoElement.videoHeight);
        //ctx.putImageData(frame.imagedata, 0, 0);
}


