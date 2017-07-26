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


var accel_sensor = null;
var orientation_sensor = null;
var gyroscope = null;
var sensorfreq = 60;
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
var frameData = {"frame": null, "data": null, "time": null, "ori": null, "aVel": null, "accel": null, "accelnog": null, "timeDiff": null};
var dataArray = [];     //array to store all the combined data
var dataArray2 = [];     //array to store all the combined data

var velocity = {"x": 0, "y": 0, "z": 0};

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
var timeAtStart = null;
var nFrame = 0; //frame number with which we can combine timestamp and frame data
var prevFrame = null;      //previous frame
var delay = 0;
var sensorframeTimeDiff = 0;    //time difference between sensor and frame data in ms - this is how much the timestamps differ

//canvas
var canvas = document.getElementById('myCanvas');
var canvas2 = document.getElementById('myCanvas2');
//CSS.elementSources.set("pfcanvas", canvas);
var ctx = canvas.getContext('2d');
var ctx2 = canvas2.getContext('2d');
//var ctx = document.getCSSCanvasContext("2d", "name_of_canvas", 100, 100); - polyfill from https://css-tricks.com/media-fragments-uri-spatial-dimension/

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

var cameraPath2 = [];    //array of canvas coordinates describing the camera path
var cameraCoord = {"x": null, "y": null, "time": null};

//Sliders
var slider_delay = document.getElementById("slider_delay");
var slider_delay_div = document.getElementById("slider_delay_amount");
slider_delay.onchange = () => {
        delay = slider_delay.value;
        slider_delay_div.innerHTML = delay;
        console.log("Delay:", delay);
};

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
                toEulerianAngle(sensor.quaternion, euler);      //From quaternion to Eulerian angles
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

function update_debug()
{
                        document.getElementById("ori").textContent = `Orientation: ${ori.roll} ${ori.pitch} ${ori.yaw}`;
                        document.getElementById("accel").textContent = `Acceleration: ${accel.x} ${accel.y} ${accel.z}, M ${magnitude(accel).toFixed(3)}`;
                        document.getElementById("accelnog").textContent = `Linear acceleration (no gravity): ${accelNoG.x} ${accelNoG.y} ${accelNoG.z}, M ${magnitude(accelNoG).toFixed(3)}`;
                        document.getElementById("rrate").textContent = `Rotation rate: ${aVel.x} ${aVel.y} ${aVel.z}, M ${magnitude(aVel).toFixed(3)} and alpha ${alpha} beta ${beta} gamma ${gamma}`;
                        document.getElementById("selectedSensor").textContent = `${selectedSensor}`;
}

function selectSensor() {
        selectedSensor = document.getElementById("selectSensor").value;
        console.log(selectedSensor, "selected");
}

function buildCameraPath(dataArray) {    //Build the shaky camera path from the sensor measurements (convert to canvas coordinates) using projection
        let cameraPath = [];
        for (let i=0; i<dataArray.length; i++)
        {
                let ori = dataArray[i].ori;
                let oriDiff = null;
                if(ori !== undefined)
                {
                        oriDiff = {"roll": ori.roll-oriInitial.roll, "pitch": ori.pitch-oriInitial.pitch, "yaw": ori.yaw-oriInitial.yaw};
                        cameraCoord.x = (1/2) * canvas.width + (1/2)*Math.sin(oriDiff.yaw) * canvas.width;
                        cameraCoord.y = (1/2) * canvas.height + (1/2)*Math.sin(oriDiff.roll) * canvas.height;
                        var b = new Object;     //need to push by value
                        Object.assign(b, cameraCoord);
                        cameraPath.push(b);
                }
        }
        console.log(cameraPath);
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
                timeInitial = Date.now();
                const bias = 0.98;
                //Initialize sensors
                accel_sensor = new Accelerometer({frequency: sensorfreq});
                // Remove drift with a high pass filter.
                const accel_filtered =  new HighPassFilterData(accel_sensor, 0.8);
                //console.log(accel_filtered);  //null, null, null
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
                const gyro_filtered = new LowPassFilterData(gyroscope, 0.1);
                gyroscope.onreading = () => {
                        //console.log(Date.now());
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
                        //time = orientation_sensor.timestamp;
                        //time = Date.now();
                        if(!initialoriobtained) //obtain initial orientation
                        {
                                oriInitial = {"roll": orientation_sensor.roll, "pitch": orientation_sensor.pitch, "yaw": orientation_sensor.yaw, "time": orientation_sensor.timestamp};
                                //timeAtStart = orientation_sensor.timestamp;
                                timeAtStart = Date.now();
                                initialoriobtained = true;
                                sensorframeTimeDiff = timeAtStart - timeInitial;
                                console.log(sensorframeTimeDiff);
                        }
                        ori = {"roll": orientation_sensor.roll, "pitch": orientation_sensor.pitch, "yaw": orientation_sensor.yaw, "time": orientation_sensor.timestamp};
                        //console.log(orientation_sensor);
                        ori_filtered.update(ori);
                        //console.log(ori_filtered);
                        //ori = ori_filtered;
                        //ori.roll = ori_filtered.roll;
                        //ori.pitch = ori_filtered.pitch;
                        //ori.yaw = ori_filtered.yaw;
                        //ori.time = orientation_sensor.timestamp;
                        //console.log(ori_filtered);
                };
                orientation_sensor.onactivate = () => {
                };
                orientation_sensor.start();
                }
                catch(err) {
                        console.log(err.message);
                        console.log("Your browser doesn't seem to support generic sensors. If you are running Chrome, please enable it in about:flags.");
                        //this.innerHTML = "Your browser doesn't seem to support generic sensors. If you are running Chrome, please enable it in about:flags";
                }
}

document.addEventListener('DOMContentLoaded', function() {
    startSensors();     //start sensors instantly to avoid gyro drift
}, false);

function startRecording(stream) {
                interval=window.setInterval(update_debug,100);
	        //var options = {mimeType: 'video/webm;codecs=vp9'};
		//mediaRecorder = new MediaRecorder(stream, options);
		mediaRecorder = new MediaRecorder(stream);

	        mediaRecorder.start(1000/fps);  //argument blob length in ms
                frame = 0;

	        var url = window.URL;
	        videoElement.src = url ? url.createObjectURL(stream) : stream;	        
                //videoElement.play();

	        mediaRecorder.ondataavailable = function(e) {
                        //console.log("Data available", e);
                        //console.log(time);
                        frameData.frame = frame;
                        time = Date.now();
                        timestamps.push(time);
                        frameData.time = time;
                        timestampDiffs.push(time-timeAtStart);
		        chunks.push(e.data);
                        frameData.data = e.data;         
                        orientationData.push(ori);
                        aVelData.push(aVel);
                        frameData.ori = ori;
                        frameData.aVel = aVel;
                        frameData.accel = accel;
                        frameData.timeDiff = time-timeAtStart;
                        //frameData.accelnog = accelNoG;
                        //dataArray.push(frameData);
                        var b = new Object;     //need to push by value
                        Object.assign(b, frameData);
                        dataArray.push(b);
                        frameData = {"frame": null, "data": null, "time": null, "ori": null, "aVel": null, "accel": null, "accelnog": null, "timeDiff": null};
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
//canvas.style.display="none";
  canvas2.width = videoElement.videoWidth;
  canvas2.height = videoElement.videoHeight;
        ctx.save();     //save canvas state for later restoration
        //duration = videoElement.duration;
});
/*
videoElement.addEventListener('durationchange', function() {
        duration = videoElement.duration;
});
*/
        duration = (dataArray[dataArray.length-1].time - dataArray[0].time)/1000;      //duration in s
        //videoElement.play();
videoElement.addEventListener('play', function() { 
        videoElement.play();
        nFrame = 0;
        let durationPerFrame = duration*1000/dataArray.length;   //frame duration in ms
        //delay = Math.floor(sensorframeTimeDiff/durationPerFrame);
        //console.log("Delay", delay);
        cameraPath2 = buildCameraPath(dataArray);     //build camera path
        //cameraPath = smooth(cameraPath, 0.85);       //smoothen the path
        //console.log(cameraPath2);
        readFrameData();    //reads the video into dataArray2
}, false);

                        /*//Read blob data so we can stabilize the video                        
                        var reader = new FileReader();
                         reader.onload = function(event){
                                let text = reader.result;
                                console.log(text);
                          };
                        reader.readAsText(blob, "video/webm");*/
                        //console.log(orientationData);
                        //console.log(timestamps);
                        //console.log(dataArray);
/*
var blobUrl = URL.createObjectURL(blob);
var x = new XMLHttpRequest();
// set x.responseType = 'blob' if you want to get a Blob object:
// x.responseType = 'blob';
x.onload = function() {
    alert(x.responseText);
};
console.log(x.open('get', blobUrl));*/
                        //readFrameData(blob, orientationData);    //reads the video into dataArray2
                        //console.log(dataArray);
                        /*videoElement.onended = function() {
                                alert("The video has ended");
                                cancelAnimationFrame(ref);
                        };*/
                        //ctx.clearRect(0, 0, canvas.width, canvas.height);
                        //stabilize(dataArray2);        //now we can operate on it
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
        //console.log("frame");
        nFrame = videoElement.webkitDecodedFrameCount - extraFrames;
        //console.log(prevFrame, nFrame);
        //let x = 0;
        //let y = 0;
        let dx = 0;
        let dy = 0;
        //let delay = -10;
        var timeFromStart = null;
        //var cameraPos = null;
        let frameDataL = (nFrame-delay >=0 && nFrame-delay <= dataArray.length) ? dataArray[nFrame - delay] : dataArray[nFrame];
        let frame = frameDataL.frame;
        console.log(frame);
        nFrame = frame;
        if(nFrame === 0 && !videoElement.ended)
        {
                //console.log(dataArray);
                //timeAtStart = frameDataL.ori.time;
        }
        else if(nFrame !== 0 && nFrame !== prevFrame && frameDataL !== undefined)    //all subsequent frames
        {
                //console.log(nFrame);
                //console.log(dataL);
                timeFromStart = frameDataL.time - timeAtStart; //time since recording start (in ms)
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
                                //angle = oriDiff.yaw;
                                angle = {"alpha":frameDataL.aVel.alpha - dataArray[0].aVel.alpha, "beta":frameDataL.aVel.beta, "gamma":frameDataL.aVel.gamma - dataArray[0].aVel.gamma};
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
        //let trans = {"x": x+0.1*canvas.width + widthR/2, "y": y+0.1*canvas.height + heightR/2};
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        //ctx.translate(videoElement.videoWidth/2 + videoElement.videoWidth * (2*angle.gamma/(Math.PI)), videoElement.videoHeight/2);
        ctx.translate(videoElement.videoWidth/2 + (1/4) * videoElement.videoWidth * Math.sin(angle.gamma), videoElement.videoHeight/2);
        ctx.rotate(angle.alpha);
        ctx.drawImage(videoElement,-videoElement.videoWidth/2,-videoElement.videoHeight/2, videoElement.videoWidth, videoElement.videoHeight);      
        ctx.rotate(-angle.alpha);
        //ctx.translate(-(videoElement.videoWidth/2 + videoElement.videoWidth * (2*angle.gamma/(Math.PI)), -videoElement.videoHeight/2);
        ctx.translate(-(videoElement.videoWidth/2 + (1/4) * videoElement.videoWidth * Math.sin(angle.gamma)), -videoElement.videoHeight/2);
        ctx.beginPath();
        ctx.rect((canvas.width-widthR)/2,(canvas.height-heightR)/2,widthR,heightR);
        var imgData=ctx.getImageData((canvas.width-widthR)/2,(canvas.height-heightR)/2,widthR,heightR);
        ctx.stroke();
        //ctx2.scale(1/0.8, 1/0.8);
        //ctx2.translate(-videoElement.videoWidth/2 - videoElement.videoWidth * (1.5*angle.gamma/(Math.PI)), 0);
        ctx2.putImageData(imgData, 0, 0, 0, 0, canvas2.width, canvas2.height);
        }
        if(videoElement.ended)
        {
                //x = 0;
                //y = 0;
                console.log("ended");
                cancelAnimationFrame(ref);
        console.log(cameraPath2);
        if(cameraPath2 !== undefined) {
        //console.log(cameraPath);
        for(let i=0; i<nFrame; i++)
        {
                ctx.fillRect(cameraPath2[i].x,cameraPath2[i].y,3,3);
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
