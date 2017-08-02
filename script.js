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
var frameData = {"frame": null, "data": null, "time": null, "ori": null, "aVel": null, "accel": null, "accelnog": null, "timeDiff": null};
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
var timeAtStart = null;
var nFrame = 0; //frame number with which we can combine timestamp and frame data
var prevFrame = null;      //previous frame
var delay = 0;
var smoothing = 0.85;
var rco = 1;    //determines canvas rotation amount
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

var cameraPath = [];    //smoothed camera path
var cameraPath2 = [];    //array of canvas coordinates describing the camera path
var cameraCoord = {"x": null, "y": null, "time": null};

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

function hannWindow(dataIn) {   //Low-pass filter with Hanning window of length dataIn.length TODO: Should split into smaller lengths
        let dataOut = [];
        for (let i = 0; i < dataIn.length; i++) {
                let multiplier = 0.5 * (1 + Math.cos(2*Math.PI*i/(dataIn.length-1))); //the weight
                //dataOut[i] = multiplier * dataIn[i];
                dataOut.push(multiplier * dataIn[i]);
        }    
        return dataOut;   
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

function buildCameraPath(dataArray) {    //Build the shaky camera path from the sensor measurements (convert to canvas coordinates) using projection
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
                const gyro_filtered = new LowPassFilterData(gyroscope, 0.85);
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
                        if(!initialoriobtained && recordingStarted) //obtain initial orientation
                        {
                                oriInitial = {"roll": orientation_sensor.roll, "pitch": orientation_sensor.pitch, "yaw": orientation_sensor.yaw, "time": orientation_sensor.timestamp};
                                //timeAtStart = orientation_sensor.timestamp;
                                timeAtStart = Date.now();
                                initialoriobtained = true;
                                sensorframeTimeDiff = timeAtStart - timeInitial;
                                console.log("Initial orientation obtained");
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
                        nosensors = true;
                }
}

document.addEventListener('DOMContentLoaded', function() {
    //Read URL params
    urlParams = new URLSearchParams(window.location.search);
    nosensors = urlParams.has('nosensors'); //to specify whether or not to use sensors in the URL
    console.log(nosensors);
    startSensors();     //start sensors instantly to avoid gyro drift
}, false);

function startRecording(stream) {
                recordingStarted = true;
                interval=window.setInterval(update_debug,100);
	        //var options = {mimeType: 'video/webm;codecs=vp9'};
		//mediaRecorder = new MediaRecorder(stream, options);
		mediaRecorder = new MediaRecorder(stream);

	        mediaRecorder.start(1000/(2*fps));  //argument blob length in ms
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
                        frameData.accel = accel;        //maybe should use filtered acceleration instead?
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
        let tempCameraPath = {"x": null, "y": null};
        //delay = Math.floor(sensorframeTimeDiff/durationPerFrame);
        //console.log("Delay", delay);
        //Hanning window, first process data
        if(true === true)
        {
            //Obtained from phone with sensors
            let quatArray =[{"0":0.9728366435609761,"1":-0.2007873275549215,"2":0.11520988277739302,"3":0,"4":0.18622287868464266,"5":0.38315104789882326,"6":-0.9047189526245332,"7":0,"8":0.1375132991773263,"9":0.9015984654362386,"10":0.41013457051432234,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9740777319606522,"1":-0.20035827423375763,"2":0.1050196839538231,"3":0,"4":0.1754062969982323,"5":0.3758139070954196,"6":-0.9099431507320093,"7":0,"8":0.14284678171053922,"9":0.9047764743497693,"10":0.4012160612455109,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9740777319606522,"1":-0.20035827423375763,"2":0.1050196839538231,"3":0,"4":0.1754062969982323,"5":0.3758139070954196,"6":-0.9099431507320093,"7":0,"8":0.14284678171053922,"9":0.9047764743497693,"10":0.4012160612455109,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9746946261752274,"1":-0.20045134056257252,"2":0.0989426285930819,"3":0,"4":0.1695589233534105,"5":0.3745241812008362,"6":-0.9115817896962797,"7":0,"8":0.14567139457516443,"9":0.9052904770631953,"10":0.39903502226686816,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9748857170762032,"1":-0.2007510711544933,"2":0.09642014728323334,"3":0,"4":0.16778793355999166,"5":0.37739103736670954,"6":-0.9107267333325781,"7":0,"8":0.14644127397514906,"9":0.9040326215824761,"10":0.40159674071640905,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9750871506539666,"1":-0.20147739352661698,"2":0.09280037864762236,"3":0,"4":0.1651867904328903,"5":0.3803127164210789,"6":-0.9099865963419947,"7":0,"8":0.14804855502807768,"9":0.9026456343179913,"10":0.40411944528290056,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9753004995181074,"1":-0.20203581365165668,"2":0.08927747675189757,"3":0,"4":0.16252294783467924,"5":0.38266863295168185,"6":-0.9094784571948821,"7":0,"8":0.14958351941373316,"9":0.9015244326564699,"10":0.4060523627161431,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9754129944278794,"1":-0.20232789190911116,"2":0.08736654965705304,"3":0,"4":0.16130860315370588,"5":0.3853350437441482,"6":-0.9085683392924419,"7":0,"8":0.1501633257222399,"9":0.9003223404823634,"10":0.4084980389439852,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9754129944278794,"1":-0.20232789190911116,"2":0.08736654965705304,"3":0,"4":0.16130860315370588,"5":0.3853350437441482,"6":-0.9085683392924419,"7":0,"8":0.1501633257222399,"9":0.9003223404823634,"10":0.4084980389439852,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9754457186381682,"1":-0.20247684169894953,"2":0.0866532095240089,"3":0,"4":0.16111451649440411,"5":0.3877729597772497,"6":-0.9075650103977324,"7":0,"8":0.15015912466436276,"9":0.8992414937596465,"10":0.4108734317198639,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9755901669581731,"1":-0.20211172520894882,"2":0.08587593537347837,"3":0,"4":0.16055036741969397,"5":0.38965611275253564,"6":-0.9068581385925266,"7":0,"8":0.149824581643363,"9":0.8985092957466624,"10":0.4125937842181602,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9757802566942736,"1":-0.20141643754704397,"2":0.08534814612967434,"3":0,"4":0.1600786034066406,"5":0.39155491641511275,"6":-0.9061233391038936,"7":0,"8":0.14908966460923434,"9":0.8978396759016991,"10":0.4143140220598407,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9759664999864929,"1":-0.2003609297019744,"2":0.08570230668761081,"3":0,"4":0.16036637097397977,"5":0.3940514867362318,"6":-0.9049894848035694,"7":0,"8":0.14755342958824347,"9":0.8969831873526698,"10":0.4167122087906361,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9760423945093776,"1":-0.19961771194332067,"2":0.08656795918890148,"3":0,"4":0.16137537387310052,"5":0.39727894022486465,"6":-0.9033976926484908,"7":0,"8":0.14594256125933547,"9":0.8957243836436592,"10":0.41997447259933585,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9760423945093776,"1":-0.19961771194332067,"2":0.08656795918890148,"3":0,"4":0.16137537387310052,"5":0.39727894022486465,"6":-0.9033976926484908,"7":0,"8":0.14594256125933547,"9":0.8957243836436592,"10":0.41997447259933585,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9759797761774704,"1":-0.19996062908147438,"2":0.08648249870805769,"3":0,"4":0.16205021521015395,"5":0.40098889972982643,"6":-0.901636075093985,"7":0,"8":0.14561319828844832,"9":0.8939930821756309,"10":0.42376072029240774,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9757737735998986,"1":-0.20117859512071234,"2":0.08598088455466169,"3":0,"4":0.16275085730285,"5":0.4048301258265621,"6":-0.8997914487481458,"7":0,"8":0.14621114215658437,"9":0.891986359569672,"10":0.42776461139971245,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9756319587363362,"1":-0.2020153029574674,"2":0.08562766740276029,"3":0,"4":0.16339253720913316,"5":0.40845595043352023,"6":-0.8980348490624727,"7":0,"8":0.14644165677223508,"9":0.8901424204683758,"10":0.431510472894736,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9756275810685209,"1":-0.2011037064370349,"2":0.0877959107718802,"3":0,"4":0.165620682137833,"5":0.41239676111148593,"6":-0.8958229134237125,"7":0,"8":0.14394656070631195,"9":0.8885303606653158,"10":0.43565258788323324,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9756275810685209,"1":-0.2011037064370349,"2":0.0877959107718802,"3":0,"4":0.165620682137833,"5":0.41239676111148593,"6":-0.8958229134237125,"7":0,"8":0.14394656070631195,"9":0.8885303606653158,"10":0.43565258788323324,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9757796438985361,"1":-0.19824598768557045,"2":0.09248033812372713,"3":0,"4":0.16936612277942964,"5":0.41707535848535526,"6":-0.8929519663100849,"7":0,"8":0.13845288378206289,"9":0.8869873877620562,"10":0.44054980102242347,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9761243622591919,"1":-0.19371624381582242,"2":0.098261129419565,"3":0,"4":0.1735548486584837,"5":0.4235461688790262,"6":-0.8890936037661106,"7":0,"8":0.13061373273913612,"9":0.8849196226960772,"10":0.4470541241188446,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9761243622591919,"1":-0.19371624381582242,"2":0.098261129419565,"3":0,"4":0.1735548486584837,"5":0.4235461688790262,"6":-0.8890936037661106,"7":0,"8":0.13061373273913612,"9":0.8849196226960772,"10":0.4470541241188446,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9769648255439672,"1":-0.18552797725881412,"2":0.10544713452090865,"3":0,"4":0.17768013215144116,"5":0.43351631354248965,"6":-0.8834553462708347,"7":0,"8":0.11819263582416656,"9":0.8818406590025926,"10":0.4564948262716817,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9787720905417003,"1":-0.17052505914634575,"2":0.11369432540254099,"3":0,"4":0.1797377973516493,"5":0.447607770298887,"6":-0.8759803793800555,"7":0,"8":0.09848613927239225,"9":0.8778203147904381,"10":0.46875579095462183,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9809441607561845,"1":-0.15218795745168223,"2":0.1207782264055437,"3":0,"4":0.1787042337297089,"5":0.4627886042800763,"6":-0.8682692725460124,"7":0,"8":0.07624533568724967,"9":0.8733072531515977,"10":0.48116641047950837,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9830440295946558,"1":-0.13305819775759398,"2":0.12617428889122406,"3":0,"4":0.17501925703861998,"5":0.4755603985298159,"6":-0.8620965984374482,"7":0,"8":0.0547055283618989,"9":0.8695618444833411,"10":0.49078455807513044,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9830440295946558,"1":-0.13305819775759398,"2":0.12617428889122406,"3":0,"4":0.17501925703861998,"5":0.4755603985298159,"6":-0.8620965984374482,"7":0,"8":0.0547055283618989,"9":0.8695618444833411,"10":0.49078455807513044,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9848921524470378,"1":-0.11343662873944371,"2":0.13084181438014286,"3":0,"4":0.16983282868712113,"5":0.4851440607409393,"6":-0.8577832403905175,"7":0,"8":0.033826903940341824,"9":0.8670452170763385,"10":0.49707984079734346,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9862189667349064,"1":-0.0950710731343225,"2":0.1354017747534879,"3":0,"4":0.1648027202086606,"5":0.49245606652001683,"6":-0.8545917635411389,"7":0,"8":0.01456753119550136,"9":0.8651291868580184,"10":0.501337484632824,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9870149571208545,"1":-0.07876218019432635,"2":0.13999283365331294,"3":0,"4":0.1606032544135072,"5":0.4993077569737857,"6":-0.8514096321503768,"7":0,"8":-0.002840629338361822,"9":0.8628373462117214,"10":0.5054736867419061,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.987482766681027,"1":-0.062493800329099125,"2":0.1448182016103723,"3":0,"4":0.15640288638984656,"5":0.5066896384212177,"6":-0.8478229711286613,"7":0,"8":-0.020394208152328286,"9":0.8598605573957516,"10":0.5101214960557892,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.987482766681027,"1":-0.062493800329099125,"2":0.1448182016103723,"3":0,"4":0.15640288638984656,"5":0.5066896384212177,"6":-0.8478229711286613,"7":0,"8":-0.020394208152328286,"9":0.8598605573957516,"10":0.5101214960557892,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9877019579162614,"1":-0.04593125001290099,"2":0.14944953518430348,"3":0,"4":0.15161151629583935,"5":0.5148799925343415,"6":-0.8437491212005361,"7":0,"8":-0.03819412809619327,"9":0.8560309291564487,"10":0.5155116826700006,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9877645284168312,"1":-0.030305286356424377,"2":0.15297982048588965,"3":0,"4":0.14599011804510442,"5":0.5246659027332664,"6":-0.8386969385848342,"7":0,"8":-0.054846342541835735,"9":0.8507686283457165,"10":0.5226706405501076,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9880093438089272,"1":-0.01937703009796854,"2":0.15317332228057845,"3":0,"4":0.1395249645612764,"5":0.5368509948476416,"6":-0.8320599562018737,"7":0,"8":-0.0661083974852068,"9":0.8434545139810871,"10":0.5331173787842682,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9886376180779585,"1":-0.013463463233760464,"2":0.14971438312164542,"3":0,"4":0.13234491824524852,"5":0.5502314568225304,"6":-0.8244575279588737,"7":0,"8":-0.07127751377819047,"9":0.8349036638745524,"10":0.5457613374311155,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9892287245649021,"1":-0.009285006352111491,"2":0.1460832593510546,"3":0,"4":0.12578297479358858,"5":0.5643580446521255,"6":-0.8158913045657933,"7":0,"8":-0.07486770503803153,"9":0.8254779017460357,"10":0.5594470845624371,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9892287245649021,"1":-0.009285006352111491,"2":0.1460832593510546,"3":0,"4":0.12578297479358858,"5":0.5643580446521255,"6":-0.8158913045657933,"7":0,"8":-0.07486770503803153,"9":0.8254779017460357,"10":0.5594470845624371,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9900864738384506,"1":-0.007051573632966157,"2":0.14028203168298803,"3":0,"4":0.11835590717936695,"5":0.5796890006879181,"6":-0.8061963098790206,"7":0,"8":-0.07563499414660568,"9":0.8148072693196222,"10":0.5747768343157319,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.991406367669342,"1":-0.006786554306507098,"2":0.1306420910306283,"3":0,"4":0.10894704702096125,"5":0.5956432058949578,"6":-0.7958264075638086,"7":0,"8":-0.0724151518235141,"9":0.8032204384296688,"10":0.5912638424492918,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9928191400123914,"1":-0.00806535003888409,"2":0.11935285670336038,"3":0,"4":0.09929420804419276,"5":0.6119884685295485,"6":-0.7846086433843489,"7":0,"8":-0.06671442515213011,"9":0.7908255263667847,"10":0.6083947078051348,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9941201050833679,"1":-0.010531297579070342,"2":0.10776970378172179,"3":0,"4":0.09039770327964569,"5":0.6286167598510419,"6":-0.772443695802914,"7":0,"8":-0.0596110100876015,"9":0.7776439414765609,"10":0.6258725643105685,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9952315768813562,"1":-0.014252528125645814,"2":0.09649338928981477,"3":0,"4":0.08287831950035551,"5":0.6451976416402085,"6":-0.7595072178025543,"7":0,"8":-0.05143241090406958,"9":0.7638827758101417,"10":0.6433022917620452,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9952315768813562,"1":-0.014252528125645814,"2":0.09649338928981477,"3":0,"4":0.08287831950035551,"5":0.6451976416402085,"6":-0.7595072178025543,"7":0,"8":-0.05143241090406958,"9":0.7638827758101417,"10":0.6433022917620452,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9962823909253983,"1":-0.018902097294243747,"2":0.08404824967593949,"3":0,"4":0.07564874662829912,"5":0.6587224156798868,"6":-0.7485733420270905,"7":0,"8":-0.04121486005408148,"9":0.7521485837106382,"10":0.6577034632100514,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9971326713126061,"1":-0.02422394888216539,"2":0.0716912529153213,"3":0,"4":0.06946681581963832,"5":0.6687625018097911,"6":-0.7402236394733243,"7":0,"8":-0.030013279459178976,"9":0.7430813382368094,"10":0.6685277006625903,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9971326713126061,"1":-0.02422394888216539,"2":0.0716912529153213,"3":0,"4":0.06946681581963832,"5":0.6687625018097911,"6":-0.7402236394733243,"7":0,"8":-0.030013279459178976,"9":0.7430813382368094,"10":0.6685277006625903,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.997463256581057,"1":-0.029242937641440703,"2":0.06489917132993661,"3":0,"4":0.0675172039094527,"5":0.6774817863435482,"6":-0.7324342054494807,"7":0,"8":-0.022549479316894216,"9":0.7349580183481649,"10":0.6777375902254308,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9975514847905426,"1":-0.03254471713714935,"2":0.06190215454704773,"3":0,"4":0.06738350345449451,"5":0.6842141685963536,"6":-0.7261614527432031,"7":0,"8":-0.018721613533607906,"9":0.7285546193487377,"10":0.6847318367613965,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9976051571563213,"1":-0.03510149771325333,"2":0.05959727624175848,"3":0,"4":0.06734224494501317,"5":0.6894991824002829,"6":-0.7211490189680854,"7":0,"8":-0.015778863169311208,"9":0.7234353947458976,"10":0.6902117538342831,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9976051571563213,"1":-0.03510149771325333,"2":0.05959727624175848,"3":0,"4":0.06734224494501317,"5":0.6894991824002829,"6":-0.7211490189680854,"7":0,"8":-0.015778863169311208,"9":0.7234353947458976,"10":0.6902117538342831,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9976424071890695,"1":-0.03726557229072236,"2":0.05762728729870903,"3":0,"4":0.06734062985750822,"5":0.6933920017451329,"6":-0.7174069541594673,"7":0,"8":-0.013223716949370856,"9":0.7195962586254981,"10":0.694266753033882,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9975186551531462,"1":-0.039128599349130266,"2":0.058527647454452714,"3":0,"4":0.06923786309907942,"5":0.6958269311651257,"6":-0.714864338531206,"7":0,"8":-0.012753474704981338,"9":0.717142842746939,"10":0.6968095250730526,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9972163584351613,"1":-0.039881674612107676,"2":0.06299989404103956,"3":0,"4":0.07293692371472293,"5":0.69721849013451,"6":-0.7131385548827427,"7":0,"8":-0.015483532480701334,"9":0.7157484511553958,"10":0.6981865303234944,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9970917297663443,"1":-0.04056705232339897,"2":0.06451663837140631,"3":0,"4":0.07447172298205484,"5":0.6984567808989974,"6":-0.7117668722073419,"7":0,"8":-0.016187799017787774,"9":0.714501527053322,"10":0.6994465781637567,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9971178086306393,"1":-0.041668571765653084,"2":0.06340193660012527,"3":0,"4":0.07440622271338704,"5":0.700376116935323,"6":-0.7098851881567341,"7":0,"8":-0.014825297945118443,"9":0.7125566619363028,"10":0.7014579001551475,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9971178086306393,"1":-0.041668571765653084,"2":0.06340193660012527,"3":0,"4":0.07440622271338704,"5":0.700376116935323,"6":-0.7098851881567341,"7":0,"8":-0.014825297945118443,"9":0.7125566619363028,"10":0.7014579001551475,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9971683075587541,"1":-0.042373882659697215,"2":0.06212745140471965,"3":0,"4":0.07395432434269245,"5":0.7024206855901034,"6":-0.7079095366073982,"7":0,"8":-0.013642729225467942,"9":0.7104995483299252,"10":0.7035653782820439,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.997213584427313,"1":-0.04293976417479761,"2":0.061001999527521056,"3":0,"4":0.07352544943134298,"5":0.7039909392177508,"6":-0.7063928039113544,"7":0,"8":-0.012612516785397654,"9":0.7089096993160969,"10":0.7051864931752528,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9972026962035664,"1":-0.04394165582816756,"2":0.06046415236299785,"3":0,"4":0.07383210791480144,"5":0.7051153059590376,"6":-0.7052384287111501,"7":0,"8":-0.011644856321744168,"9":0.7077298583309833,"10":0.7063871892001253,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9971641433894803,"1":-0.04450643382712283,"2":0.06068647735918198,"3":0,"4":0.07437864817362172,"5":0.7056944735773154,"6":-0.7046013991148663,"7":0,"8":-0.011466816565166127,"9":0.707117028706532,"10":0.7070035535874614,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9971641433894803,"1":-0.04450643382712283,"2":0.06068647735918198,"3":0,"4":0.07437864817362172,"5":0.7056944735773154,"6":-0.7046013991148663,"7":0,"8":-0.011466816565166127,"9":0.707117028706532,"10":0.7070035535874614,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9971483715454953,"1":-0.044838969778309146,"2":0.06070083973066143,"3":0,"4":0.07462282366829287,"5":0.7057518082428422,"6":-0.7045181491695947,"7":0,"8":-0.01124985966838854,"9":0.7070387932149127,"10":0.7070852768174037,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9970683976253204,"1":-0.04567893942062584,"2":0.061384406318579776,"3":0,"4":0.075698919223123,"5":0.7057754580459814,"6":-0.7043796671325462,"7":0,"8":-0.011148294673572368,"9":0.7069614390719712,"10":0.7071642510960884,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9969657271271504,"1":-0.04651466320162845,"2":0.06241574548616269,"3":0,"4":0.07702547769895807,"5":0.7053096822530058,"6":-0.7047023141415129,"7":0,"8":-0.011243440870199795,"9":0.7073716575311703,"10":0.706752394190644,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9968874141345759,"1":-0.04724016185573299,"2":0.06311775288459787,"3":0,"4":0.07804249638472505,"5":0.7047763658081252,"6":-0.7051238548407355,"7":0,"8":-0.011173736143170299,"9":0.7078549632624345,"10":0.7062694289543324,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9968280309356382,"1":-0.04736911508592545,"2":0.06395345137520092,"3":0,"4":0.078725898655855,"5":0.7046975313256602,"6":-0.7051266885985161,"7":0,"8":-0.011666614232582084,"9":0.7079248413702242,"10":0.70619142851471,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9968280309356382,"1":-0.04736911508592545,"2":0.06395345137520092,"3":0,"4":0.078725898655855,"5":0.7046975313256602,"6":-0.7051266885985161,"7":0,"8":-0.011666614232582084,"9":0.7079248413702242,"10":0.70619142851471,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9968267733651436,"1":-0.04733590306166091,"2":0.06399762612074955,"3":0,"4":0.07873771650603745,"5":0.7044553029113783,"6":-0.7053673475354716,"7":0,"8":-0.011694266422870747,"9":0.7081680840387745,"10":0.7059470275224337,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9968093485544445,"1":-0.04827219708114783,"2":0.06356821239079147,"3":0,"4":0.07909972240657204,"5":0.7041016283274101,"6":-0.7056799070349888,"7":0,"8":-0.010693762331621004,"9":0.7084565563721363,"10":0.7056734028359015,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9967096059897017,"1":-0.04961741822238086,"2":0.06409425379723954,"3":0,"4":0.08042794198842695,"5":0.703598604646094,"6":-0.7060314232941483,"7":0,"8":-0.010065173043015019,"9":0.7088632705537787,"10":0.7052741146702746,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9967096059897017,"1":-0.04961741822238086,"2":0.06409425379723954,"3":0,"4":0.08042794198842695,"5":0.703598604646094,"6":-0.7060314232941483,"7":0,"8":-0.010065173043015019,"9":0.7088632705537787,"10":0.7052741146702746,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9966407120934445,"1":-0.05122875589591391,"2":0.06389761678714057,"3":0,"4":0.08144500577947866,"5":0.70192057186789,"6":-0.7075833588638347,"7":0,"8":-0.008602435428417321,"9":0.7104105244693399,"10":0.7037349450043145,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.996532615579584,"1":-0.05269028936523856,"2":0.06439316403766515,"3":0,"4":0.08285096241847345,"5":0.6995297288667199,"6":-0.7097843923115486,"7":0,"8":-0.007646188233814577,"9":0.712658332547476,"10":0.7014696315032616,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.996532615579584,"1":-0.05269028936523856,"2":0.06439316403766515,"3":0,"4":0.08285096241847345,"5":0.6995297288667199,"6":-0.7097843923115486,"7":0,"8":-0.007646188233814577,"9":0.712658332547476,"10":0.7014696315032616,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9963436051099224,"1":-0.05377277155552562,"2":0.06639208829043719,"3":0,"4":0.08506425656359795,"5":0.696828359366243,"6":-0.7121757455658809,"7":0,"8":-0.00796822462097646,"9":0.7152193435277553,"10":0.6988546207113464,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.996159841704726,"1":-0.054744809394877136,"2":0.06832697574580648,"3":0,"4":0.08716882825304195,"5":0.6931884947377105,"6":-0.7154657963728769,"7":0,"8":-0.008195434780481792,"9":0.7186742768756941,"10":0.6952985820808798,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9959834932138294,"1":-0.055776191669187014,"2":0.0700421154090991,"3":0,"4":0.08917218203672395,"5":0.6884582755333437,"6":-0.719773258472226,"7":0,"8":-0.008074864637181989,"9":0.723128092459281,"10":0.6906667627265723,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.995712869073072,"1":-0.05639255068106763,"2":0.07331959063401783,"3":0,"4":0.09203224363425022,"5":0.6834381793213813,"6":-0.7241838889334005,"7":0,"8":-0.009270829234903744,"9":0.7278269843079059,"10":0.6856981234969924,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9954290873897254,"1":-0.05655169709428143,"2":0.07695997562626689,"3":0,"4":0.09486847868687698,"5":0.6782940977921306,"6":-0.7286405909392918,"7":0,"8":-0.010995637511145961,"9":0.7326111141447961,"10":0.6805586479826364,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9954290873897254,"1":-0.05655169709428143,"2":0.07695997562626689,"3":0,"4":0.09486847868687698,"5":0.6782940977921306,"6":-0.7286405909392918,"7":0,"8":-0.010995637511145961,"9":0.7326111141447961,"10":0.6805586479826364,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9952089202021216,"1":-0.05669121223114104,"2":0.07965746552291475,"3":0,"4":0.09702984326194375,"5":0.6728227124275361,"6":-0.7334131314190808,"7":0,"8":-0.012017273997594335,"9":0.7376284418846273,"10":0.67509990440642,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9949605554595051,"1":-0.057164176952485324,"2":0.08237566120739892,"3":0,"4":0.09944937983377367,"5":0.6673308852134527,"6":-0.7380916504753634,"7":0,"8":-0.012779418480080906,"9":0.7425642871321736,"10":0.6696528504614445,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9946618099946558,"1":-0.05759418694919416,"2":0.08562005114294391,"3":0,"4":0.1022590946782792,"5":0.6612827925324927,"6":-0.7431339936546064,"7":0,"8":-0.013818866944789177,"9":0.7479224322074631,"10":0.663642268906325,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9943608495072659,"1":-0.05757682555916188,"2":0.08905846089507174,"3":0,"4":0.1049579345307855,"5":0.6544863043541191,"6":-0.7487532760657153,"7":0,"8":-0.015176701799156644,"9":0.7538783360719684,"10":0.6568387027498013,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9940354154324654,"1":-0.056887260791083194,"2":0.09304532620019934,"3":0,"4":0.1076763280644325,"5":0.6473115130108968,"6":-0.7545817592255893,"7":0,"8":-0.01730322347352997,"9":0.7600997714386658,"10":0.649575979897299,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9940354154324654,"1":-0.056887260791083194,"2":0.09304532620019934,"3":0,"4":0.1076763280644325,"5":0.6473115130108968,"6":-0.7545817592255893,"7":0,"8":-0.01730322347352997,"9":0.7600997714386658,"10":0.649575979897299,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9936120183740467,"1":-0.056067276281472456,"2":0.09793680418079065,"3":0,"4":0.11105248960043435,"5":0.6400775357678781,"6":-0.7602421329113064,"7":0,"8":-0.020062443428329413,"9":0.7662618459967611,"10":0.6422151427528786,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9930801609487041,"1":-0.055442035735213446,"2":0.1035276525495703,"3":0,"4":0.1151746669342486,"5":0.6320452231706728,"6":-0.7663247644838851,"7":0,"8":-0.02294755467308307,"9":0.7729456832392767,"10":0.6340570897990414,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9924898034567575,"1":-0.054700410121387044,"2":0.10941597046861684,"3":0,"4":0.11956250589600703,"5":0.6228698869267766,"6":-0.7731350996776121,"7":0,"8":-0.02586110293286925,"9":0.7804107510165375,"10":0.6247321301204884,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9920111755797562,"1":-0.05399117757480898,"2":0.11401219311446953,"3":0,"4":0.12308147431194083,"5":0.6123724581843176,"6":-0.7809295222762616,"7":0,"8":-0.027654621962426074,"9":0.7887236023002713,"10":0.6141256359268232,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9920111755797562,"1":-0.05399117757480898,"2":0.11401219311446953,"3":0,"4":0.12308147431194083,"5":0.6123724581843176,"6":-0.7809295222762616,"7":0,"8":-0.027654621962426074,"9":0.7887236023002713,"10":0.6141256359268232,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9915764509631976,"1":-0.05362278950786692,"2":0.11790138834054087,"3":0,"4":0.1263268868367904,"5":0.6013479347139031,"6":-0.7889373443221563,"7":0,"8":-0.028594730097638354,"9":0.7971858077320286,"10":0.6030564515418604,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9908722653454639,"1":-0.053222280963422186,"2":0.12385290590965448,"3":0,"4":0.13127564131547054,"5":0.5898018357574386,"6":-0.7968064307810185,"7":0,"8":-0.030640814128007454,"9":0.8057922628977177,"10":0.5914050703298316,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9898337041823794,"1":-0.051906335376953106,"2":0.132419677683683,"3":0,"4":0.13793766195726453,"5":0.5773258723348387,"6":-0.8047782704282176,"7":0,"8":-0.03467621813114352,"9":0.8148623168900408,"10":0.5786164449237167,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9885383940289993,"1":-0.04932990472163157,"2":0.14268288304957766,"3":0,"4":0.14548655132003718,"5":0.5636757026955059,"6":-0.8130826568469691,"7":0,"8":-0.04031758921595996,"9":0.8245218638583689,"10":0.5643919110344828,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9885383940289993,"1":-0.04932990472163157,"2":0.14268288304957766,"3":0,"4":0.14548655132003718,"5":0.5636757026955059,"6":-0.8130826568469691,"7":0,"8":-0.04031758921595996,"9":0.8245218638583689,"10":0.5643919110344828,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9872292034267532,"1":-0.04640076900769663,"2":0.15239904025992868,"3":0,"4":0.15268146711695807,"5":0.5485964211498374,"6":-0.8220281617233305,"7":0,"8":-0.045462824765584386,"9":0.8347987168789366,"10":0.5486749458776079,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9872292034267532,"1":-0.04640076900769663,"2":0.15239904025992868,"3":0,"4":0.15268146711695807,"5":0.5485964211498374,"6":-0.8220281617233305,"7":0,"8":-0.045462824765584386,"9":0.8347987168789366,"10":0.5486749458776079,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9859152548707264,"1":-0.04389921478850001,"2":0.1613814404173035,"3":0,"4":0.15976828884723604,"5":0.5325161515777388,"6":-0.8312043355272367,"7":0,"8":-0.04944900655257589,"9":0.8452806708197,"10":0.5320294996582433,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9843901384040501,"1":-0.041680526766176396,"2":0.17099353035358233,"3":0,"4":0.16776894918159746,"5":0.5158638027338716,"6":-0.8400821773249731,"7":0,"8":-0.053194298799683404,"9":0.8556560165493252,"10":0.5148039391300312,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9828474740990807,"1":-0.03938676412200648,"2":0.18016526922133314,"3":0,"4":0.1756073906880622,"5":0.49826426600237794,"6":-0.8490551981007558,"7":0,"8":-0.05632837435367999,"9":0.8661301102595272,"10":0.4966343827084123,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9813123460167613,"1":-0.03663628677283981,"2":0.1889017251436531,"3":0,"4":0.18306613171759523,"5":0.4801381719523685,"6":-0.8578776832058097,"7":0,"8":-0.05926947518500647,"9":0.8764274701400623,"10":0.47787237935570637,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9798121303804332,"1":-0.03317567702977353,"2":0.19714858590285456,"3":0,"4":0.19001139402803258,"5":0.46114784024580213,"6":-0.8667400847753924,"7":0,"8":-0.06215995985683076,"9":0.8867029259399942,"10":0.45814200385412895,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9785228459849201,"1":-0.029156549125961995,"2":0.20406600554488197,"3":0,"4":0.1958511459462473,"5":0.44032009186234944,"6":-0.8762194483292878,"7":0,"8":-0.06430682395764986,"9":0.8973673097689736,"10":0.4365736133671074,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9785228459849201,"1":-0.029156549125961995,"2":0.20406600554488197,"3":0,"4":0.1958511459462473,"5":0.44032009186234944,"6":-0.8762194483292878,"7":0,"8":-0.06430682395764986,"9":0.8973673097689736,"10":0.4365736133671074,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9772698195340772,"1":-0.02488603525413602,"2":0.2105335654671503,"3":0,"4":0.20141173373329657,"5":0.4189040225505063,"6":-0.88541101165064,"7":0,"8":-0.06615897863123799,"9":0.9076893914148121,"10":0.41439459918418287,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9756959969009235,"1":-0.020776819953427506,"2":0.21814134533313378,"3":0,"4":0.208169564500567,"5":0.39876316881032536,"6":-0.8931144362861465,"7":0,"8":-0.06843065984462626,"9":0.9168185685508883,"10":0.3933967286293907,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9738671837937333,"1":-0.017570293778540247,"2":0.22643761831743436,"3":0,"4":0.2160176307817565,"5":0.37954171516811364,"6":-0.8996024201543396,"7":0,"8":-0.07013624886015091,"9":0.9250077923632016,"10":0.3734187137962621,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.972072873867857,"1":-0.01476148676716349,"2":0.2342144844166576,"3":0,"4":0.22362778434726494,"5":0.3609224120211163,"6":-0.9053869824211276,"7":0,"8":-0.07116839375784068,"9":0.9324789929956196,"10":0.3541439655152985,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.972072873867857,"1":-0.01476148676716349,"2":0.2342144844166576,"3":0,"4":0.22362778434726494,"5":0.3609224120211163,"6":-0.9053869824211276,"7":0,"8":-0.07116839375784068,"9":0.9324789929956196,"10":0.3541439655152985,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9703606819016599,"1":-0.01156349345517782,"2":0.24138440003646622,"3":0,"4":0.23059547769202737,"5":0.34310152697175766,"6":-0.910553116479818,"7":0,"8":-0.0722901696625371,"9":0.9392270960115212,"10":0.3355987224860968,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9687973559274504,"1":-0.008221687941768252,"2":0.24771775294121934,"3":0,"4":0.23681939560116838,"5":0.3256030011405908,"6":-0.9153683515561881,"7":0,"8":-0.07313176561492352,"9":0.9454708081220615,"10":0.3173903824462937,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9670127734892738,"1":-0.004964350460190836,"2":0.2546795117718368,"3":0,"4":0.24369008773189993,"5":0.30915408712357895,"6":-0.9192599989429442,"7":0,"8":-0.07417169023995385,"9":0.9509990324600079,"10":0.3001657132844099,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9650779771171537,"1":-0.001704347390655947,"2":0.2619572336331384,"3":0,"4":0.25081977259374266,"5":0.2945604168649075,"6":-0.922129915831637,"7":0,"8":-0.07559059947359792,"9":0.9556313280735467,"10":0.2847012556415751,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9630251208181843,"1":0.0009592956366759608,"2":0.26940990669154186,"3":0,"4":0.258232650444878,"5":0.28178375279864987,"6":-0.9240745868536908,"7":0,"8":-0.0768017989104246,"9":0.9594774742413119,"10":0.2711171106311592,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9630251208181843,"1":0.0009592956366759608,"2":0.26940990669154186,"3":0,"4":0.258232650444878,"5":0.28178375279864987,"6":-0.9240745868536908,"7":0,"8":-0.0768017989104246,"9":0.9594774742413119,"10":0.2711171106311592,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9609811771239822,"1":0.003247788641711935,"2":0.2765947052591198,"3":0,"4":0.2654749564119161,"5":0.2700707264253842,"6":-0.9255187116945717,"7":0,"8":-0.07770602696401241,"9":0.9628350274307781,"10":0.2586706773682178,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9592807864358279,"1":0.005703959988951901,"2":0.2823965942438944,"3":0,"4":0.27134463048988255,"5":0.25903372165196525,"6":-0.9269701188122809,"7":0,"8":-0.0784376383148766,"9":0.9658514246544286,"10":0.2469383337171357,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9580012362164009,"1":0.008117563163536623,"2":0.28664915120456147,"3":0,"4":0.27577513439537693,"5":0.2479867484665479,"6":-0.9286822037070765,"7":0,"8":-0.07862382569693072,"9":0.9687294077302011,"10":0.23533298976485006,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9547832783425887,"1":0.01238848337822418,"2":0.2970444723555872,"3":0,"4":0.2865437076612878,"5":0.22800807643722087,"6":-0.9305401922339237,"7":0,"8":-0.0792565245083594,"9":0.9735804390201216,"10":0.21414845616727873,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9547832783425887,"1":0.01238848337822418,"2":0.2970444723555872,"3":0,"4":0.2865437076612878,"5":0.22800807643722087,"6":-0.9305401922339237,"7":0,"8":-0.0792565245083594,"9":0.9735804390201216,"10":0.21414845616727873,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9530906519865442,"1":0.014404131605789239,"2":0.3023420653619837,"3":0,"4":0.2921129825272031,"5":0.21791453029867647,"6":-0.9312267014745008,"7":0,"8":-0.07929822780950957,"9":0.9758615091767378,"10":0.20348466985472757,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9530906519865442,"1":0.014404131605789239,"2":0.3023420653619837,"3":0,"4":0.2921129825272031,"5":0.21791453029867647,"6":-0.9312267014745008,"7":0,"8":-0.07929822780950957,"9":0.9758615091767378,"10":0.20348466985472757,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.951375365204126,"1":0.015710085486499104,"2":0.3076330725445757,"3":0,"4":0.2978342360245181,"5":0.20795553181765747,"6":-0.9316916112931874,"7":0,"8":-0.07861095318656908,"9":0.9780121082667694,"10":0.19316476885043699,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9491296279083694,"1":0.0170535372261309,"2":0.3144234742435543,"3":0,"4":0.3049711419483554,"5":0.19880906868919368,"6":-0.9313793404649111,"7":0,"8":-0.07839353842011576,"9":0.9798898153028297,"10":0.18349474246370745,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9463981151545763,"1":0.019264629525354238,"2":0.322427497872841,"3":0,"4":0.3131198260621044,"5":0.19031642650619396,"6":-0.9304492349955937,"7":0,"8":-0.07928803101431825,"9":0.9815338396745672,"10":0.17408296649992705,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9433112592960525,"1":0.02179679532502199,"2":0.3311929491647376,"3":0,"4":0.32194004372855467,"5":0.1826378543634507,"6":-0.9289768823967015,"7":0,"8":-0.08073709250978567,"9":0.9829386244812022,"10":0.16526708253819145,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9398459463424285,"1":0.024012626481436428,"2":0.3407535609434156,"3":0,"4":0.3315483783334514,"5":0.17606512343730252,"6":-0.9268639182629528,"7":0,"8":-0.08225125136199196,"9":0.9840855877019834,"10":0.15751274574471807,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9398459463424285,"1":0.024012626481436428,"2":0.3407535609434156,"3":0,"4":0.3315483783334514,"5":0.17606512343730252,"6":-0.9268639182629528,"7":0,"8":-0.08225125136199196,"9":0.9840855877019834,"10":0.15751274574471807,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.936295852471202,"1":0.02553408310939531,"2":0.35028286973185274,"3":0,"4":0.34117249310687114,"5":0.17063875231826664,"6":-0.924382911329622,"7":0,"8":-0.08337510524767922,"9":0.9850027651792397,"10":0.15105682872476178,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9324780644291506,"1":0.026630146306949598,"2":0.3602436610330493,"3":0,"4":0.35119303432901816,"5":0.16658287170574182,"6":-0.9213650782918159,"7":0,"8":-0.08454651172935002,"9":0.9856677889454324,"10":0.14598255166378094,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9284812830713118,"1":0.027102717430398826,"2":0.37038892112721555,"3":0,"4":0.3614167162264543,"5":0.16351378265325245,"6":-0.9179549212993514,"7":0,"8":-0.08544277276622836,"9":0.9861687092042111,"10":0.14202411046411356,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9251987849356023,"1":0.027692570625857726,"2":0.3784710496552144,"3":0,"4":0.3696699945060242,"5":0.15956599405057403,"6":-0.9153594108131742,"7":0,"8":-0.08573977523240828,"9":0.9867988028643211,"10":0.1373931494321492,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9218494775654378,"1":0.02864040263456502,"2":0.3864883733301312,"3":0,"4":0.3778358340529593,"5":0.15544444623348852,"6":-0.9127305456933783,"7":0,"8":-0.08621843125425954,"9":0.9874293361416591,"10":0.13247501301542997,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9218494775654378,"1":0.02864040263456502,"2":0.3864883733301312,"3":0,"4":0.3778358340529593,"5":0.15544444623348852,"6":-0.9127305456933783,"7":0,"8":-0.08621843125425954,"9":0.9874293361416591,"10":0.13247501301542997,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9175867716244137,"1":0.02943773910983971,"2":0.3964441174427442,"3":0,"4":0.3878256519195862,"5":0.15277640881489418,"6":-0.9089833085533492,"7":0,"8":-0.08732572617507017,"9":0.9878222568010635,"10":0.12876890060432045,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9134403830630293,"1":0.028850210931285147,"2":0.40594867529848244,"3":0,"4":0.39735567248412806,"5":0.15233979810604836,"6":-0.9049314930196903,"7":0,"8":-0.08794959460997376,"9":0.9879069808657999,"10":0.12768953033924246,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9095590657836166,"1":0.03133900952938973,"2":0.41439133402397843,"3":0,"4":0.4056652726089567,"5":0.14950136954636228,"6":-0.9017123031827587,"7":0,"8":-0.0902108520720013,"9":0.9882647709970094,"10":0.12326717626363859,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9050847440825893,"1":0.03518924246707478,"2":0.4237727184987321,"3":0,"4":0.41473562655010277,"5":0.1469980636856434,"6":-0.8979899068069113,"7":0,"8":-0.09389334267830662,"9":0.9885106119165261,"10":0.11845147453167071,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9050847440825893,"1":0.03518924246707478,"2":0.4237727184987321,"3":0,"4":0.41473562655010277,"5":0.1469980636856434,"6":-0.8979899068069113,"7":0,"8":-0.09389334267830662,"9":0.9885106119165261,"10":0.11845147453167071,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.900112923647522,"1":0.03950001257869573,"2":0.43386228297678286,"3":0,"4":0.4243707259694176,"5":0.14571321193527664,"6":-0.8936874206495502,"7":0,"8":-0.09852014273097964,"9":0.9885380457490442,"10":0.11439569382670056,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.8956463178104435,"1":0.04398803803646878,"2":0.4425864050310082,"3":0,"4":0.432667750606063,"5":0.14435483094598522,"6":-0.8899215136735572,"7":0,"8":-0.10303538709617754,"9":0.9885477911680587,"10":0.11025866731873002,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.892295299508818,"1":0.048665072328608616,"2":0.44882157234926723,"3":0,"4":0.43856600071088714,"5":0.14239613445100785,"6":-0.887346133640663,"7":0,"8":-0.10709321216246792,"9":0.9886126686431984,"10":0.10571655707509642,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.8897021214764633,"1":0.053857985846013534,"2":0.45335356281354855,"3":0,"4":0.442680091973747,"5":0.14106517556828235,"6":-0.8855139504385741,"7":0,"8":-0.1116443985394806,"9":0.9885342369446923,"10":0.10166412767625088,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.8882827280075443,"1":0.05932865407676502,"2":0.45544911950556255,"3":0,"4":0.44420966248057425,"5":0.14109972327539566,"6":-0.8847421103306026,"7":0,"8":-0.11675429504667889,"9":0.988216037533693,"10":0.09898208741337378,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.8882827280075443,"1":0.05932865407676502,"2":0.45544911950556255,"3":0,"4":0.44420966248057425,"5":0.14109972327539566,"6":-0.8847421103306026,"7":0,"8":-0.11675429504667889,"9":0.988216037533693,"10":0.09898208741337378,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.8882827280075443,"1":0.05932865407676502,"2":0.45544911950556255,"3":0,"4":0.44420966248057425,"5":0.14109972327539566,"6":-0.8847421103306026,"7":0,"8":-0.11675429504667889,"9":0.988216037533693,"10":0.09898208741337378,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.888014727104987,"1":0.06388476117892239,"2":0.4553554317290889,"3":0,"4":0.4433882371565687,"5":0.14336364842102567,"6":-0.884790167002691,"7":0,"8":-0.12180600587517532,"9":0.9876059466014628,"10":0.09898328327080064,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.8887479490365684,"1":0.06671131796647067,"2":0.45351592069295776,"3":0,"4":0.44069500381193016,"5":0.1479321102571216,"6":-0.8853835662512859,"7":0,"8":-0.1261546823625963,"9":0.9867450257426276,"10":0.1020750129508694,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.8905788801351058,"1":0.0680348734212366,"2":0.4497115774394622,"3":0,"4":0.43594634939281107,"5":0.15425130731685321,"6":-0.8866550776590554,"7":0,"8":-0.12969204808165458,"9":0.9856864117385715,"10":0.10771340509813676,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.8935671183244329,"1":0.06879726611005843,"2":0.44362679369628033,"3":0,"4":0.42880893150296906,"5":0.16172713095737334,"6":-0.8888009695027423,"7":0,"8":-0.13289355203598952,"9":0.9844344565797738,"10":0.1150131667576555,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.89720549408595,"1":0.07063131362621355,"2":0.43592835411729247,"3":0,"4":0.4199544312011749,"5":0.16888629310341763,"6":-0.8916926514791212,"7":0,"8":-0.13660376313437972,"9":0.9831015850474474,"10":0.12186377392361392,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.89720549408595,"1":0.07063131362621355,"2":0.43592835411729247,"3":0,"4":0.4199544312011749,"5":0.16888629310341763,"6":-0.8916926514791212,"7":0,"8":-0.13660376313437972,"9":0.9831015850474474,"10":0.12186377392361392,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9014993852644152,"1":0.07296353696749591,"2":0.42658548757054326,"3":0,"4":0.40952438182594264,"5":0.17494789499694885,"6":-0.8953675191384103,"7":0,"8":-0.13995941062566963,"9":0.9818704272515721,"10":0.12783507305584685,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.906544787481868,"1":0.07409086104112816,"2":0.4155563760521126,"3":0,"4":0.39772628514985797,"5":0.17982398001224276,"6":-0.8997095186684403,"7":0,"8":-0.1413872677021324,"9":0.9809046641707271,"10":0.13355060652735729,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9128978750920113,"1":0.0726374691168612,"2":0.40167310162993886,"3":0,"4":0.3836879462735201,"5":0.18308707454838113,"6":-0.9051313735826301,"7":0,"8":-0.1392876246373902,"9":0.9804096295118012,"10":0.1392696767736492,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9204430925038856,"1":0.06822378713212096,"2":0.384876646099956,"3":0,"4":0.36756486165529445,"5":0.18388496802912568,"6":-0.9116372282657528,"7":0,"8":-0.13296838090359664,"9":0.980577318906108,"10":0.14417898066141355,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9288217780277233,"1":0.060522912282912955,"2":0.36555038193779765,"3":0,"4":0.34986664689296987,"5":0.18157569010650398,"6":-0.9190340570282416,"7":0,"8":-0.1219976804613303,"9":0.9815127333227593,"10":0.14747650695471037,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9288217780277233,"1":0.060522912282912955,"2":0.36555038193779765,"3":0,"4":0.34986664689296987,"5":0.18157569010650398,"6":-0.9190340570282416,"7":0,"8":-0.1219976804613303,"9":0.9815127333227593,"10":0.14747650695471037,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9372967235483634,"1":0.05005069922123795,"2":0.3449199623337229,"3":0,"4":0.3316613383769429,"5":0.17608816593500576,"6":-0.9268191441972218,"7":0,"8":-0.10712427145538328,"9":0.9831011631220465,"10":0.14844697884913627,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9452309901820567,"1":0.037166446764333116,"2":0.32427923164058825,"3":0,"4":0.3140452415439068,"5":0.16721986264311794,"6":-0.9345656457733105,"7":0,"8":-0.08896039063608896,"9":0.985218765163844,"10":0.14638945380300328,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9517395621263581,"1":0.021012475505545858,"2":0.3061866782131837,"3":0,"4":0.29932307967503835,"5":0.15686038520359957,"6":-0.9411697595534443,"7":0,"8":-0.06780486719952705,"9":0.9873972342462614,"10":0.14300071539221504,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9567040592479321,"1":0.0026268909457467515,"2":0.29105058708793585,"3":0,"4":0.28755317300707084,"5":0.14627983777900821,"6":-0.9465280856296769,"7":0,"8":-0.04506126378430331,"9":0.9892397806579427,"10":0.13919114301770064,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9602936021284083,"1":-0.01620246340525977,"2":0.27852050549641216,"3":0,"4":0.27806819352853207,"5":0.13673159050810735,"6":-0.9507799027316576,"7":0,"8":-0.022677556088665618,"9":0.9904755544031577,"10":0.13580786418929314,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9602936021284083,"1":-0.01620246340525977,"2":0.27852050549641216,"3":0,"4":0.27806819352853207,"5":0.13673159050810735,"6":-0.9507799027316576,"7":0,"8":-0.022677556088665618,"9":0.9904755544031577,"10":0.13580786418929314,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9626651382052129,"1":-0.03447449630736621,"2":0.26849085646838056,"3":0,"4":0.27068998192400606,"5":0.12869069189244042,"6":-0.9540260065984874,"7":0,"8":-0.0016627052512117046,"9":0.9910853629655869,"10":0.13321794391681108,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9636661227861593,"1":-0.05001495348988971,"2":0.2623854240931296,"3":0,"4":0.2666593026655102,"5":0.12311460118790218,"6":-0.9558952206884184,"7":0,"8":0.015505568389166946,"9":0.9911313540900724,"10":0.1319783214165251,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.963678801022127,"1":-0.06288208364429071,"2":0.259555413194799,"3":0,"4":0.2654921313603138,"5":0.12027936502365177,"6":-0.9565807936001354,"7":0,"8":0.028932631196078695,"9":0.9907465518766438,"10":0.13260537234887415,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.963678801022127,"1":-0.06288208364429071,"2":0.259555413194799,"3":0,"4":0.2654921313603138,"5":0.12027936502365177,"6":-0.9565807936001354,"7":0,"8":0.028932631196078695,"9":0.9907465518766438,"10":0.13260537234887415,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9636740596792883,"1":-0.07474463233233308,"2":0.2564089408188508,"3":0,"4":0.26393289865743697,"5":0.11954200424066919,"6":-0.9571045360132135,"7":0,"8":0.04088679494193315,"9":0.9900115695224425,"10":0.13492709721427154,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9636740596792883,"1":-0.07474463233233308,"2":0.2564089408188508,"3":0,"4":0.26393289865743697,"5":0.11954200424066919,"6":-0.9571045360132135,"7":0,"8":0.04088679494193315,"9":0.9900115695224425,"10":0.13492709721427154,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.963765442337875,"1":-0.08499365485881416,"2":0.2528482757623651,"3":0,"4":0.26183743323688,"5":0.12032653295893947,"6":-0.9575816823461764,"7":0,"8":0.050964011598211734,"9":0.9890892772863084,"10":0.1382210748658571,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9633945284349152,"1":-0.10170953841518049,"2":0.24804465710988133,"3":0,"4":0.25971329432699797,"5":0.12463480743800737,"6":-0.9576090844459776,"7":0,"8":0.06648298184652823,"9":0.9869758475560667,"10":0.14648781118893683,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9622406716188285,"1":-0.1201279526714325,"2":0.24425839473266553,"3":0,"4":0.2591077259647383,"5":0.12928201717176036,"6":-0.9571568901880038,"7":0,"8":0.0834030860087962,"9":0.98430452659528,"10":0.1555264969207566,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9614376344872095,"1":-0.12584791161944597,"2":0.24454034727625507,"3":0,"4":0.2606896170278459,"5":0.13371146332018524,"6":-0.9561183203888955,"7":0,"8":0.08762763193674905,"9":0.9829972645691502,"10":0.16136247403764026,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9610825345155538,"1":-0.11834552250799879,"2":0.24962913147719723,"3":0,"4":0.2651813517005963,"5":0.14185921553489989,"6":-0.9537058795869652,"7":0,"8":0.07745461051477642,"9":0.9827870529014976,"10":0.1677214365992863,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9610825345155538,"1":-0.11834552250799879,"2":0.24962913147719723,"3":0,"4":0.2651813517005963,"5":0.14185921553489989,"6":-0.9537058795869652,"7":0,"8":0.07745461051477642,"9":0.9827870529014976,"10":0.1677214365992863,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9610809783975868,"1":-0.10083157569823697,"2":0.25720876042163443,"3":0,"4":0.2703041439163343,"5":0.1508182098859664,"6":-0.9508888159255386,"7":0,"8":0.057087852272446504,"9":0.9834057472949942,"10":0.1722037053631169,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9613046891644864,"1":-0.07806619005649473,"2":0.26419493215513246,"3":0,"4":0.27368138446565915,"5":0.1609932783713529,"6":-0.9482508172555728,"7":0,"8":0.03149272937285463,"9":0.9838631930298871,"10":0.17612885805519873,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9608594935318422,"1":-0.05094532584172384,"2":0.27231160643198393,"3":0,"4":0.2770352444252353,"5":0.17416055208874326,"6":-0.9449442576075455,"7":0,"8":0.0007145410675803987,"9":0.9833985714928133,"10":0.18145746857430112,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.958828282111955,"1":-0.02202025024840637,"2":0.28313147546086803,"3":0,"4":0.2820296359623633,"5":0.19067353120442432,"6":-0.9402674392932402,"7":0,"8":-0.03328074903885714,"9":0.9814064812987164,"10":0.18903353830065217,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.955824572875307,"1":0.004646153113437634,"2":0.2939010030173357,"3":0,"4":0.28652117073115946,"5":0.20846416815796287,"6":-0.9351194131998244,"7":0,"8":-0.06561253718760796,"9":0.9780189729660589,"10":0.19792395309790178,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.955824572875307,"1":0.004646153113437634,"2":0.2939010030173357,"3":0,"4":0.28652117073115946,"5":0.20846416815796287,"6":-0.9351194131998244,"7":0,"8":-0.06561253718760796,"9":0.9780189729660589,"10":0.19792395309790178,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9530289753363004,"1":0.025087622778450225,"2":0.30183834104425955,"3":0,"4":0.2888709139356038,"5":0.22427980620110333,"6":-0.9307266926198032,"7":0,"8":-0.09104596653907038,"9":0.9742018232667782,"10":0.20649806913223578,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9503096953750165,"1":0.043513043155336106,"2":0.3082500492281657,"3":0,"4":0.28998854821669795,"5":0.23637977662518317,"6":-0.927378652274041,"7":0,"8":-0.11321713508553621,"9":0.9706859110267372,"10":0.2120157106046956,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9485221281551075,"1":0.05715495678839311,"2":0.3115109753234915,"3":0,"4":0.2891969695219263,"5":0.24468007322933216,"6":-0.9254711446710311,"7":0,"8":-0.12911579938880458,"9":0.9679178878984991,"10":0.21555542251794035,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9484413123042557,"1":0.06410559767377855,"2":0.31040224139482575,"3":0,"4":0.28589801726332986,"5":0.24973788236853833,"6":-0.9251450388774112,"7":0,"8":-0.1368261774695938,"9":0.9661891593168894,"10":0.21853406122020935,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9499512661429494,"1":0.06405492979673255,"2":0.3057606304178684,"3":0,"4":0.28101911936224866,"5":0.252312215688157,"6":-0.9259410769171961,"7":0,"8":-0.13645824158937003,"9":0.9655234794340561,"10":0.2216836477868145,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9499512661429494,"1":0.06405492979673255,"2":0.3057606304178684,"3":0,"4":0.28101911936224866,"5":0.252312215688157,"6":-0.9259410769171961,"7":0,"8":-0.13645824158937003,"9":0.9655234794340561,"10":0.2216836477868145,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.952437779646686,"1":0.05854150253850143,"2":0.2990571353571152,"3":0,"4":0.2757673267209899,"5":0.25201992536888784,"6":-0.9275981713236867,"7":0,"8":-0.12967135088610782,"9":0.9659497287180236,"10":0.22388946411482946,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9552630600290927,"1":0.05048158915045953,"2":0.29141738418969787,"3":0,"4":0.2706795780169369,"5":0.24784399826582504,"6":-0.930218178625507,"7":0,"8":-0.11918493485208081,"9":0.9674837998424712,"10":0.22309188172452066,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9581604607672558,"1":0.03907069623399195,"2":0.2835524826683269,"3":0,"4":0.2667021629909101,"5":0.2377842369305081,"6":-0.9339853222202534,"7":0,"8":-0.10391576390303214,"9":0.9705318678759043,"10":0.21741521525916196,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9581604607672558,"1":0.03907069623399195,"2":0.2835524826683269,"3":0,"4":0.2667021629909101,"5":0.2377842369305081,"6":-0.9339853222202534,"7":0,"8":-0.10391576390303214,"9":0.9705318678759043,"10":0.21741521525916196,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9607936249942791,"1":0.02301691570115505,"2":0.27630750416561156,"3":0,"4":0.26460258396726655,"5":0.22159766042769746,"6":-0.9385521033123556,"7":0,"8":-0.08283168369935012,"9":0.9748665547990694,"10":0.20681928249545034,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9607936249942791,"1":0.02301691570115505,"2":0.27630750416561156,"3":0,"4":0.26460258396726655,"5":0.22159766042769746,"6":-0.9385521033123556,"7":0,"8":-0.08283168369935012,"9":0.9748665547990694,"10":0.20681928249545034,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9628373660384563,"1":-0.01244165326721447,"2":0.269795122588083,"3":0,"4":0.26743390482581564,"5":0.18345762761819617,"6":-0.9459504877004699,"7":0,"8":-0.037726773754946485,"9":0.9829488409348264,"10":0.17996718052570726,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9628373660384563,"1":-0.01244165326721447,"2":0.269795122588083,"3":0,"4":0.26743390482581564,"5":0.18345762761819617,"6":-0.9459504877004699,"7":0,"8":-0.037726773754946485,"9":0.9829488409348264,"10":0.17996718052570726,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9628442330588246,"1":-0.029589190100885787,"2":0.26843148746225776,"3":0,"4":0.26957532371969606,"5":0.16466990637197376,"6":-0.9487955438411679,"7":0,"8":-0.01612849879708822,"9":0.9859048224857256,"10":0.1665279848061405,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9625450088277717,"1":-0.0457996210439493,"2":0.2672255579294802,"3":0,"4":0.27109492953753467,"5":0.14867167251929814,"6":-0.9510016983091663,"7":0,"8":0.0038266535681277247,"9":0.9878254328170084,"10":0.1555192224904669,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.962100573188982,"1":-0.062326771148864424,"2":0.26547667729048996,"3":0,"4":0.2716805703553753,"5":0.13518943058828903,"6":-0.9528450091449976,"7":0,"8":0.02349809095266764,"9":0.9888575819395378,"10":0.1469987977737759,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9611282200926696,"1":-0.0812454728380354,"2":0.26387822479346834,"3":0,"4":0.2725056739210556,"5":0.12537389792149412,"6":-0.9539507532864108,"7":0,"8":0.044420739049001234,"9":0.9887773031164314,"10":0.14264024380303553,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9611282200926696,"1":-0.0812454728380354,"2":0.26387822479346834,"3":0,"4":0.2725056739210556,"5":0.12537389792149412,"6":-0.9539507532864108,"7":0,"8":0.044420739049001234,"9":0.9887773031164314,"10":0.14264024380303553,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9601120975070283,"1":-0.10102175489357279,"2":0.2607285311276737,"3":0,"4":0.2718750567159205,"5":0.11939610603334883,"6":-0.9548971466090244,"7":0,"8":0.06533540725737019,"9":0.987693885829291,"10":0.14209893981696808,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9593755474675927,"1":-0.12312699203704369,"2":0.25384700627764545,"3":0,"4":0.26789361876930684,"5":0.11533448804942026,"6":-0.9565202357865594,"7":0,"8":0.08849614560475771,"9":0.9856661180533699,"10":0.1436340232125115,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9580121437249747,"1":-0.14669417931853346,"2":0.2463606134178029,"3":0,"4":0.26372455964156316,"5":0.11361396565491644,"6":-0.9578837390433161,"7":0,"8":0.11252595581197156,"9":0.9826355980910737,"10":0.1475304151780037,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9562531594555774,"1":-0.1678924869604792,"2":0.2395662819687452,"3":0,"4":0.2603082050007721,"5":0.11465585631034969,"6":-0.9586936862389805,"7":0,"8":0.13348980876223848,"9":0.9791149362804319,"10":0.15334382090353627,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9545776246383368,"1":-0.18475277030783976,"2":0.23376905206477283,"3":0,"4":0.25772344769138167,"5":0.11822382331847159,"6":-0.9589586537972244,"7":0,"8":0.14953320818383964,"9":0.9756482404258698,"10":0.16046893997898937,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9545776246383368,"1":-0.18475277030783976,"2":0.23376905206477283,"3":0,"4":0.25772344769138167,"5":0.11822382331847159,"6":-0.9589586537972244,"7":0,"8":0.14953320818383964,"9":0.9756482404258698,"10":0.16046893997898937,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9528405831107198,"1":-0.1988261012933008,"2":0.22926622956816844,"3":0,"4":0.2563170041046754,"5":0.12281172944718177,"6":-0.9587590215127305,"7":0,"8":0.16246973866754955,"9":0.9723093383125304,"10":0.16798251110924722,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9525245992148695,"1":-0.20102314007154587,"2":0.22866259554625534,"3":0,"4":0.257824266835345,"5":0.13311307068351352,"6":-0.9569783243158674,"7":0,"8":0.16193681754620082,"9":0.9705001612237703,"10":0.17862212040271597,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9547341683759664,"1":-0.18916076207124632,"2":0.22956670247264732,"3":0,"4":0.2584997860441618,"5":0.1457697413934791,"6":-0.9549497303803036,"7":0,"8":0.1471751521373843,"9":0.9710660806716078,"10":0.1880693721945078,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9577964531556299,"1":-0.17227740435258454,"2":0.23010095776287898,"3":0,"4":0.2576040501608503,"5":0.15926743761571727,"6":-0.9530341265896196,"7":0,"8":0.1275386535705323,"9":0.9720876447385025,"10":0.1969251434990439,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.960376185781004,"1":-0.15088257787076031,"2":0.23433316408684135,"3":0,"4":0.25926413496468825,"5":0.1751154691793242,"6":-0.9497982569410843,"7":0,"8":0.10227263767444583,"9":0.9729178116687884,"10":0.20729516530054637,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.960376185781004,"1":-0.15088257787076031,"2":0.23433316408684135,"3":0,"4":0.25926413496468825,"5":0.1751154691793242,"6":-0.9497982569410843,"7":0,"8":0.10227263767444583,"9":0.9729178116687884,"10":0.20729516530054637,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.960376185781004,"1":-0.15088257787076031,"2":0.23433316408684135,"3":0,"4":0.25926413496468825,"5":0.1751154691793242,"6":-0.9497982569410843,"7":0,"8":0.10227263767444583,"9":0.9729178116687884,"10":0.20729516530054637,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9614793599156747,"1":-0.09287683184031437,"2":0.2587109043409308,"3":0,"4":0.27334620806862375,"5":0.22386425939101962,"6":-0.9355033900279053,"7":0,"8":0.02897047552424592,"9":0.9701848464733431,"10":0.24062839644991196,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9614793599156747,"1":-0.09287683184031437,"2":0.2587109043409308,"3":0,"4":0.27334620806862375,"5":0.22386425939101962,"6":-0.9355033900279053,"7":0,"8":0.02897047552424592,"9":0.9701848464733431,"10":0.24062839644991196,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.96046469269974,"1":-0.06928515955918524,"2":0.26964261638016307,"3":0,"4":0.27838068964661833,"5":0.2509433027570055,"6":-0.9271092962224202,"7":0,"8":-0.0034300926160542033,"9":0.9655190428857509,"10":0.2603098327753609,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.961109273490023,"1":-0.050668144659645975,"2":0.2714805814401471,"3":0,"4":0.2747703492108613,"5":0.27422724139384513,"6":-0.9215751316354446,"7":0,"8":-0.02775287691163708,"9":0.9603292181179821,"10":0.27748444715612564,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9637021515168679,"1":-0.0408588682638813,"2":0.2638346409878629,"3":0,"4":0.2640555162005578,"5":0.2917318846084953,"6":-0.9193297698889422,"7":0,"8":-0.0394062084582707,"9":0.9556270686435129,"10":0.29193165353966133,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9637021515168679,"1":-0.0408588682638813,"2":0.2638346409878629,"3":0,"4":0.2640555162005578,"5":0.2917318846084953,"6":-0.9193297698889422,"7":0,"8":-0.0394062084582707,"9":0.9556270686435129,"10":0.29193165353966133,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9667106265179121,"1":-0.0374818141211275,"2":0.25311199136526863,"3":0,"4":0.2523700714616002,"5":0.302776368713479,"6":-0.9190407276944321,"7":0,"8":-0.042189022187879566,"9":0.9523243280210969,"10":0.30215642022789335,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9682231420806006,"1":-0.03399674200556291,"2":0.24776636446880396,"3":0,"4":0.2459801871886711,"5":0.3082820235765933,"6":-0.9189428709094947,"7":0,"8":-0.04514086108734894,"9":0.950687369232265,"10":0.30684831319254513,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.968411786284248,"1":-0.02873383126989193,"2":0.2476953385709697,"3":0,"4":0.24424653744420777,"5":0.309369817783839,"6":-0.9190397055002415,"7":0,"8":-0.050221935006022456,"9":0.9505076108480188,"10":0.306615515916695,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.968411786284248,"1":-0.02873383126989193,"2":0.2476953385709697,"3":0,"4":0.24424653744420777,"5":0.309369817783839,"6":-0.9190397055002415,"7":0,"8":-0.050221935006022456,"9":0.9505076108480188,"10":0.306615515916695,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9684086017307164,"1":-0.024392119831874837,"2":0.2481729415940661,"3":0,"4":0.243573010862713,"5":0.3058396599169799,"6":-0.9203990365609787,"7":0,"8":-0.05345065788759751,"9":0.9517705724665018,"10":0.3021190175615568,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9688455647158011,"1":-0.02476673254834072,"2":0.24642418488480367,"3":0,"4":0.24251046821848643,"5":0.2968211916526311,"6":-0.9236264464138944,"7":0,"8":-0.0502687053912414,"9":0.9546118314108551,"10":0.2935800877562008,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9698142592984665,"1":-0.030786766781463193,"2":0.2418935240387583,"3":0,"4":0.2405980043839433,"5":0.2821562970167476,"6":-0.9287090033539371,"7":0,"8":-0.039659831911009746,"9":0.9588743335887884,"10":0.281046435078224,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9708058194700006,"1":-0.041839373593375306,"2":0.23618960570268133,"3":0,"4":0.23877023665149633,"5":0.2626055582354433,"6":-0.9348941913198812,"7":0,"8":-0.022909323736473475,"9":0.9639957685029432,"10":0.26492900013841414,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9714017794772108,"1":-0.05754731183177508,"2":0.23036251763380378,"3":0,"4":0.23744048549459595,"5":0.23859247376355164,"6":-0.9416451784069033,"7":0,"8":-0.000773613429746689,"9":0.9694131900638729,"10":0.24543321536724327,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9712023476916108,"1":-0.07627720084707601,"2":0.22571617692290236,"3":0,"4":0.236980478305008,"5":0.21135897872801435,"6":-0.9482445602661744,"7":0,"8":0.024622282511695737,"9":0.9744276688410463,"10":0.22334854101490975,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9712023476916108,"1":-0.07627720084707601,"2":0.22571617692290236,"3":0,"4":0.236980478305008,"5":0.21135897872801435,"6":-0.9482445602661744,"7":0,"8":0.024622282511695737,"9":0.9744276688410463,"10":0.22334854101490975,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9698178794928548,"1":-0.09689900319347,"2":0.22374955824939802,"3":0,"4":0.23831091433550267,"5":0.18254021886661131,"6":-0.9538799737124997,"7":0,"8":0.05158672062910741,"9":0.9784118147903236,"10":0.20012285722495315,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.966723953874745,"1":-0.11971576087508762,"2":0.22608169276596257,"3":0,"4":0.24301249031149386,"5":0.1536002240236587,"6":-0.9577848688378697,"7":0,"8":0.0799357539429657,"9":0.9808542511608277,"10":0.1775814426116975,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9612000472885949,"1":-0.14495535258810222,"2":0.2346964374707028,"3":0,"4":0.2533170082172198,"5":0.1270227918314486,"6":-0.9590077003046347,"7":0,"8":0.10920148970255639,"9":0.9812508454272872,"10":0.15881396710792894,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9552931135062295,"1":-0.16747615191493992,"2":0.24365305448463914,"3":0,"4":0.2629757665611123,"5":0.10466741901748122,"6":-0.9591082214130537,"7":0,"8":0.13512519610171125,"9":0.9803043265844948,"10":0.14403022920080133,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9552931135062295,"1":-0.16747615191493992,"2":0.24365305448463914,"3":0,"4":0.2629757665611123,"5":0.10466741901748122,"6":-0.9591082214130537,"7":0,"8":0.13512519610171125,"9":0.9803043265844948,"10":0.14403022920080133,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9503121986188905,"1":-0.18408327362964538,"2":0.2510379919287278,"3":0,"4":0.27045750885772524,"5":0.08892730764338408,"6":-0.9586160166588963,"7":0,"8":0.15414104230959502,"9":0.9788796043754763,"10":0.1342954089805688,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9469238022956431,"1":-0.1943940955084278,"2":0.2560200117745186,"3":0,"4":0.27545229631213974,"5":0.08010816837187207,"6":-0.9579711278430647,"7":0,"8":0.16571464405396874,"9":0.9776469633543616,"10":0.12940263316949785,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9447435132908536,"1":-0.19939014678327083,"2":0.26019850958917146,"3":0,"4":0.2799197219029905,"5":0.07760219144903457,"6":-0.9568818447055245,"7":0,"8":0.1706008332817659,"9":0.9768426100467982,"10":0.12912740052603577,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9447435132908536,"1":-0.19939014678327083,"2":0.26019850958917146,"3":0,"4":0.2799197219029905,"5":0.07760219144903457,"6":-0.9568818447055245,"7":0,"8":0.1706008332817659,"9":0.9768426100467982,"10":0.12912740052603577,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9435299894070645,"1":-0.19694638156284316,"2":0.2663893380187412,"3":0,"4":0.28778986833565323,"5":0.0889698918440498,"6":-0.9535519441542619,"7":0,"8":0.16409798385144025,"9":0.9763690087801478,"10":0.14062493669181275,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9435299894070645,"1":-0.19694638156284316,"2":0.2663893380187412,"3":0,"4":0.28778986833565323,"5":0.0889698918440498,"6":-0.9535519441542619,"7":0,"8":0.16409798385144025,"9":0.9763690087801478,"10":0.14062493669181275,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9446140820378721,"1":-0.1880971930751283,"2":0.26893062631889686,"3":0,"4":0.2911991730364445,"5":0.10247506443961596,"6":-0.9511581884934487,"7":0,"8":0.15135150292582855,"9":0.9767897951312278,"10":0.15157313619069113,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9466459851254369,"1":-0.17355237211898977,"2":0.2715528423993083,"3":0,"4":0.2941780879625684,"5":0.1212464334243033,"6":-0.948028714524298,"7":0,"8":0.1316078441657611,"9":0.9773324742030101,"10":0.16583276061019514,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9466459851254369,"1":-0.17355237211898977,"2":0.2715528423993083,"3":0,"4":0.2941780879625684,"5":0.1212464334243033,"6":-0.948028714524298,"7":0,"8":0.1316078441657611,"9":0.9773324742030101,"10":0.16583276061019514,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9489494842847542,"1":-0.15461174444621584,"2":0.2749365041334251,"3":0,"4":0.2969480983491888,"5":0.14392814962738498,"6":-0.943984351978397,"7":0,"8":0.10637997796865406,"9":0.9774353371822464,"10":0.18249220988670423,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9507476094254872,"1":-0.1328437576205843,"2":0.28005628172110697,"3":0,"4":0.30028272990637817,"5":0.1706674445223868,"6":-0.9384577423091667,"7":0,"8":0.0768717567782975,"9":0.9763325191968608,"10":0.2021523496836184,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9507476094254872,"1":-0.1328437576205843,"2":0.28005628172110697,"3":0,"4":0.30028272990637817,"5":0.1706674445223868,"6":-0.9384577423091667,"7":0,"8":0.0768717567782975,"9":0.9763325191968608,"10":0.2021523496836184,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9510365302579066,"1":-0.11230748323108664,"2":0.2879523369386767,"3":0,"4":0.30519606545775524,"5":0.1940271175625088,"6":-0.932313711215895,"7":0,"8":0.0488352410381232,"9":0.974546316858439,"10":0.2188026778957538,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9508070077401348,"1":-0.09387418484111176,"2":0.29521801057098873,"3":0,"4":0.30882019745225,"5":0.21211877433369608,"6":-0.9271654373752369,"7":0,"8":0.024415609569063967,"9":0.9727246784358243,"10":0.23067425981541934,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9511444454568577,"1":-0.07756564458095916,"2":0.2988441260126171,"3":0,"4":0.3087102546779885,"5":0.2241483511365735,"6":-0.9243675933401485,"7":0,"8":0.004713759450388544,"9":0.9714633496645908,"10":0.23714277093688407,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9509492640076255,"1":-0.06150629845263289,"2":0.30317070049351713,"3":0,"4":0.3090388156291599,"5":0.23261278245693928,"6":-0.922163933983317,"7":0,"8":-0.013802493311004582,"9":0.9706226279845926,"10":0.24021078726638467,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9501855430278296,"1":-0.04704504607072657,"2":0.3081139305051348,"3":0,"4":0.310244861109366,"5":0.23766961427622446,"6":-0.9204679409192265,"7":0,"8":-0.0299258531432276,"9":0.9702060954998446,"10":0.24042571702226745,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9493124550514359,"1":-0.035959428560921936,"2":0.31227037424182313,"3":0,"4":0.3115783838968511,"5":0.23890274011643953,"6":-0.9196980255670837,"7":0,"8":-0.04153044449295962,"9":0.9703774868788628,"10":0.2379975251913602,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9493124550514359,"1":-0.035959428560921936,"2":0.31227037424182313,"3":0,"4":0.3115783838968511,"5":0.23890274011643953,"6":-0.9196980255670837,"7":0,"8":-0.04153044449295962,"9":0.9703774868788628,"10":0.2379975251913602,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9479931250713588,"1":-0.02825050478059321,"2":0.3170346114237974,"3":0,"4":0.3144834723121397,"5":0.23673516688760798,"6":-0.9192696131803144,"7":0,"8":-0.049083413731078274,"9":0.9711634183049869,"10":0.23330762702265417,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.946661352120844,"1":-0.024247074456149598,"2":0.32131661228454167,"3":0,"4":0.3179243324195058,"5":0.23275184549005512,"6":-0.9191032148554207,"7":0,"8":-0.05250147431556673,"9":0.9722338607980469,"10":0.22804591101368465,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9458627186605995,"1":-0.02612796046846455,"2":0.3235135951861423,"3":0,"4":0.320781117000557,"5":0.22705154693598928,"6":-0.9195363275796353,"7":0,"8":-0.04942865124985474,"9":0.973532183568949,"10":0.2231409502036854,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9460681712036432,"1":-0.03518302779018967,"2":0.32205150586994336,"3":0,"4":0.3217741466227366,"5":0.21753064784581566,"6":-0.9214889316999235,"7":0,"8":-0.03763530845885299,"9":0.9754191956465545,"10":0.21711980971923417,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9460681712036432,"1":-0.03518302779018967,"2":0.32205150586994336,"3":0,"4":0.3217741466227366,"5":0.21753064784581566,"6":-0.9214889316999235,"7":0,"8":-0.03763530845885299,"9":0.9754191956465545,"10":0.21711980971923417,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9470782880528179,"1":-0.05128000779749087,"2":0.3168802112356879,"3":0,"4":0.320536690605703,"5":0.2042458746407405,"6":-0.9249539015265942,"7":0,"8":-0.01728980763423138,"9":0.9775754962017071,"10":0.2098739621728103,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9473074565582209,"1":-0.07286314859637599,"2":0.31192874286883665,"3":0,"4":0.3202035501977818,"5":0.18850378405615942,"6":-0.9284050586271988,"7":0,"8":0.008846778614106654,"9":0.9793657274075187,"10":0.2019020814041923,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9459832583234351,"1":-0.09968544265679746,"2":0.30851010800262735,"3":0,"4":0.3216756641340748,"5":0.16970706064942176,"6":-0.9315171866773193,"7":0,"8":0.040502361923428154,"9":0.9804398576626943,"10":0.1926064197017947,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9427078428266235,"1":-0.13069153993401184,"2":0.3069554501449012,"3":0,"4":0.32468681579744985,"5":0.14792371459129994,"6":-0.9341825862604314,"7":0,"8":0.0766837572846768,"9":0.9803256368241076,"10":0.1818826627236686,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9365641224211564,"1":-0.16455763585878103,"2":0.3094647399264563,"3":0,"4":0.3310003055123225,"5":0.12488925957080532,"6":-0.9353295771800734,"7":0,"8":0.11526681451280574,"9":0.9784290493835894,"10":0.1714354308416669,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9365641224211564,"1":-0.16455763585878103,"2":0.3094647399264563,"3":0,"4":0.3310003055123225,"5":0.12488925957080532,"6":-0.9353295771800734,"7":0,"8":0.11526681451280574,"9":0.9784290493835894,"10":0.1714354308416669,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9285453171047129,"1":-0.19633427948049542,"2":0.3150499128115114,"3":0,"4":0.33926626133278903,"5":0.10435779624191888,"6":-0.9348838907131143,"7":0,"8":0.15067183141827778,"9":0.9749678640316783,"10":0.1635105374350596,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9200134824586299,"1":-0.22300203730361012,"2":0.3222503459474346,"3":0,"4":0.3485880977171174,"5":0.0899672834687717,"6":-0.9329481490998548,"7":0,"8":0.17905734342596347,"9":0.9706575102152781,"10":0.16050696785041962,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9122306249890956,"1":-0.24747401947176506,"2":0.32648414795775427,"3":0,"4":0.3552687044890046,"5":0.0810174814139688,"6":-0.9312466590078305,"7":0,"8":0.20400842333569624,"9":0.9655013215884434,"10":0.16182639974167035,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9122306249890956,"1":-0.24747401947176506,"2":0.32648414795775427,"3":0,"4":0.3552687044890046,"5":0.0810174814139688,"6":-0.9312466590078305,"7":0,"8":0.20400842333569624,"9":0.9655013215884434,"10":0.16182639974167035,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9065844909004259,"1":-0.2687895275946275,"2":0.325356349728271,"3":0,"4":0.35674404998900044,"5":0.07620432686216061,"6":-0.9310889425337775,"7":0,"8":0.2254733847940673,"9":0.9601797362820564,"10":0.1649747220366089,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9014994913537588,"1":-0.28931657695811586,"2":0.32186113127805793,"3":0,"4":0.3564996593388643,"5":0.07478205137541494,"6":-0.9312978366859099,"7":0,"8":0.24537045955376602,"9":0.9543079094410885,"10":0.17055723999631833,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9014994913537588,"1":-0.28931657695811586,"2":0.32186113127805793,"3":0,"4":0.3564996593388643,"5":0.07478205137541494,"6":-0.9312978366859099,"7":0,"8":0.24537045955376602,"9":0.9543079094410885,"10":0.17055723999631833,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9011936300629618,"1":-0.2903728669168357,"2":0.32176642449053316,"3":0,"4":0.356739883862919,"5":0.0753351930016617,"6":-0.9311612078749661,"7":0,"8":0.24614363496218106,"9":0.9539434668512555,"10":0.1714791861373336,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9022275851908663,"1":-0.2854573664733344,"2":0.3232637855097966,"3":0,"4":0.3570383062038225,"5":0.07399083834337317,"6":-0.9311546543144497,"7":0,"8":0.24188640102147296,"9":0.9555309698144091,"10":0.16867579142598155,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9074067442949989,"1":-0.26102766291045754,"2":0.3293593205347989,"3":0,"4":0.36203885266612446,"5":0.08756773085097613,"6":-0.9280408318160926,"7":0,"8":0.21340307398872227,"9":0.9613513798571329,"10":0.17396170281231882,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9128367881853761,"1":-0.22979528690725104,"2":0.3375249969212568,"3":0,"4":0.3684933492028186,"5":0.10751443555549711,"6":-0.9233922733772437,"7":0,"8":0.17590238493201227,"9":0.9672821537972873,"10":0.18282116759556954,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9128367881853761,"1":-0.22979528690725104,"2":0.3375249969212568,"3":0,"4":0.3684933492028186,"5":0.10751443555549711,"6":-0.9233922733772437,"7":0,"8":0.17590238493201227,"9":0.9672821537972873,"10":0.18282116759556954,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9182461881832412,"1":-0.18720122462613098,"2":0.3489694038488862,"3":0,"4":0.3763430393554241,"5":0.13826128179707098,"6":-0.9161057755502036,"7":0,"8":0.12324715028304656,"9":0.9725428405814291,"10":0.19740976814483158,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9218833693010072,"1":-0.1514222294220131,"2":0.35665440435673457,"3":0,"4":0.37930458353543006,"5":0.16473609507003628,"6":-0.9104889216927974,"7":0,"8":0.07911440209724052,"9":0.9746452442058937,"10":0.20930261022837449,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9260200631446133,"1":-0.11675588691955952,"2":0.3589636569078225,"3":0,"4":0.37543387777285275,"5":0.18613431347943532,"6":-0.9079666496070642,"7":0,"8":0.03919499370389712,"9":0.9755624512289645,"10":0.21619822307582204,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.929868268220845,"1":-0.0854468984954142,"2":0.35783212787467633,"3":0,"4":0.3678585997697805,"5":0.2027178586744096,"6":-0.9075161295637217,"7":0,"8":0.005005474724814718,"9":0.9755020770764231,"10":0.21993328034056558,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9321260217438834,"1":-0.060272171031805044,"2":0.35708311263514503,"3":0,"4":0.36148648199120004,"5":0.21380532036424427,"6":-0.9075322906748724,"7":0,"8":-0.0216473385827598,"9":0.9750151797172535,"10":0.2210810753632444,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9321260217438834,"1":-0.060272171031805044,"2":0.35708311263514503,"3":0,"4":0.36148648199120004,"5":0.21380532036424427,"6":-0.9075322906748724,"7":0,"8":-0.0216473385827598,"9":0.9750151797172535,"10":0.2210810753632444,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9334970423730544,"1":-0.044296280123731435,"2":0.3558386029572205,"3":0,"4":0.35662826195155617,"5":0.218215640791374,"6":-0.9084042138790909,"7":0,"8":-0.03741062101541104,"9":0.9748947494899523,"10":0.21950096071734349,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9351653219517599,"1":-0.03893243859659101,"2":0.3520654705601398,"3":0,"4":0.351776662791325,"5":0.21842232081369728,"6":-0.9102444651976307,"7":0,"8":-0.04146094368448949,"9":0.9750774698107043,"10":0.2179564984654614,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9369474561154867,"1":-0.04050172826156384,"2":0.3471153686970716,"3":0,"4":0.3473781784559584,"5":0.21645785664243355,"6":-0.9124003813162052,"7":0,"8":-0.03818206791804002,"9":0.9754515183755483,"10":0.21687905235988403,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9382870882408935,"1":-0.04490614758245126,"2":0.34292971090865,"3":0,"4":0.34441597209517827,"5":0.21175093180310967,"6":-0.9146252193329856,"7":0,"8":-0.03154341262958926,"9":0.9762914992887533,"10":0.21414955520084333,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9395002654369019,"1":-0.052020397132952034,"2":0.3385751461680728,"3":0,"4":0.3419101438468255,"5":0.20270798957896075,"6":-0.917609353431575,"7":0,"8":-0.0208974844238492,"9":0.9778565080074888,"10":0.20823051144050886,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9395002654369019,"1":-0.052020397132952034,"2":0.3385751461680728,"3":0,"4":0.3419101438468255,"5":0.20270798957896075,"6":-0.917609353431575,"7":0,"8":-0.0208974844238492,"9":0.9778565080074888,"10":0.20823051144050886,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9404823070285371,"1":-0.06382437242523409,"2":0.3337955722972694,"3":0,"4":0.33981498852126535,"5":0.18914772349999964,"6":-0.9212757310842612,"7":0,"8":-0.00433683952076791,"9":0.9798722613652622,"10":0.19957856314372258,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9402643748618202,"1":-0.08061929996603823,"2":0.33076189642636145,"3":0,"4":0.3400020312411556,"5":0.17281575844643582,"6":-0.9244097082407192,"7":0,"8":0.017364400550130554,"9":0.9816492338971132,"10":0.18990322792790515,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9391059241694335,"1":-0.10038281190774256,"2":0.3286386405249484,"3":0,"4":0.3410385919669636,"5":0.15508755921840445,"6":-0.9271680281913737,"7":0,"8":0.04210396373006198,"9":0.9827874463914279,"10":0.17987805720344419,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9375373231415484,"1":-0.12197772390650208,"2":0.3257993303901765,"3":0,"4":0.3409950306582896,"5":0.13675537311929586,"6":-0.9300647139471534,"7":0,"8":0.06889236444229763,"9":0.983066334452481,"10":0.16980706330501227,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9375373231415484,"1":-0.12197772390650208,"2":0.3257993303901765,"3":0,"4":0.3409950306582896,"5":0.13675537311929586,"6":-0.9300647139471534,"7":0,"8":0.06889236444229763,"9":0.983066334452481,"10":0.16980706330501227,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9356468187847824,"1":-0.14482988018807674,"2":0.3218529686910969,"3":0,"4":0.33934100074964846,"5":0.1184768665999485,"6":-0.9331724804596817,"7":0,"8":0.09701913657001882,"9":0.9823377722971642,"10":0.15999922215757323,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9356468187847824,"1":-0.14482988018807674,"2":0.3218529686910969,"3":0,"4":0.33934100074964846,"5":0.1184768665999485,"6":-0.9331724804596817,"7":0,"8":0.09701913657001882,"9":0.9823377722971642,"10":0.15999922215757323,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9339118242077245,"1":-0.1673231380196043,"2":0.3159298573007021,"3":0,"4":0.3349545159695495,"5":0.10065142590423193,"6":-0.9368429957572957,"7":0,"8":0.12495671043355117,"9":0.9807508827175084,"10":0.1500451951695947,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9310769871146238,"1":-0.1886954028431891,"2":0.3122333971117328,"3":0,"4":0.33213241645140634,"5":0.08432252592996115,"6":-0.9394561402491315,"7":0,"8":0.15094272843776224,"9":0.9784088237825193,"10":0.14118261870816085,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9273100975654236,"1":-0.2082405590516352,"2":0.3110174548457003,"3":0,"4":0.33100023360919906,"5":0.06830639976706321,"6":-0.9411552163901722,"7":0,"8":0.17474219010728476,"9":0.9756895846290656,"10":0.13226888350783694,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9240038777144919,"1":-0.22329408960364594,"2":0.31041356800847275,"3":0,"4":0.3297768105384855,"5":0.0544296457519553,"6":-0.9424885490026059,"7":0,"8":0.19355642300563325,"9":0.973230270448907,"10":0.12393041671855642,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9214452872680896,"1":-0.23213500232051487,"2":0.3115315933923979,"3":0,"4":0.33021353947056653,"5":0.04548512067367305,"6":-0.9428097627144856,"7":0,"8":0.20468906012357468,"9":0.9716195608452305,"10":0.11856616078708893,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9214452872680896,"1":-0.23213500232051487,"2":0.3115315933923979,"3":0,"4":0.33021353947056653,"5":0.04548512067367305,"6":-0.9428097627144856,"7":0,"8":0.20468906012357468,"9":0.9716195608452305,"10":0.11856616078708893,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9192793625637332,"1":-0.23484083392907706,"2":0.31587218511807436,"3":0,"4":0.3344466578996972,"5":0.042905626747197445,"6":-0.941437492684198,"7":0,"8":0.20753526840393732,"9":0.9710864546121947,"10":0.11798398824176815,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9178045218535185,"1":-0.23429038263386603,"2":0.3205353053076827,"3":0,"4":0.33959160860693016,"5":0.04497275597250017,"6":-0.9394972360569296,"7":0,"8":0.20569979267653693,"9":0.9711259103742054,"10":0.12083924125197276,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.917179642075181,"1":-0.23222325245230913,"2":0.3238114654557389,"3":0,"4":0.344015636517101,"5":0.051383568892553466,"6":-0.9375569158293038,"7":0,"8":0.20108392768159256,"9":0.9713043238903518,"10":0.12701639335360504,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9189943631656096,"1":-0.2205208729343422,"2":0.3268331564356144,"3":0,"4":0.3507220413938139,"5":0.07852751562805649,"6":-0.9331814182982845,"7":0,"8":0.1801205630291749,"9":0.9722160533425261,"10":0.14950786830334273,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9218968757800554,"1":-0.2110272361865415,"2":0.32492099361224547,"3":0,"4":0.35038748076752,"5":0.09624077915787232,"6":-0.9316470792832356,"7":0,"8":0.16533227096937253,"9":0.9727307811532759,"10":0.16266537892008204,"11":0,"12":0,"13":0,"14":0,"15":1},{"0":0.9218968757800554,"1":-0.2110272361865415,"2":0.32492099361224547,"3":0,"4":0.35038748076752,"5":0.09624077915787232,"6":-0.9316470792832356,"7":0,"8":0.16533227096937253,"9":0.9727307811532759,"10":0.16266537892008204,"11":0,"12":0,"13":0,"14":0,"15":1}];
            let quatArray2 = [];        //windowed quaternions
            for (let i=0; i<quatArray.length; i++)
            {
                let quat = [];
                for(let j=0; j<quatArray[i].keys; j++)
                {
                    quat.push(quatArray[i].j);
                }
                quatArray2.push(quat);
            }
            console.log(quatArray2);
            hannWindow(quatArray2);
        }
        cameraPath = buildCameraPath(dataArray);     //build camera path
        console.log(cameraPath);
        let weightedIndex = 10;
        tempCameraPath.x = doWMA(cameraPath.map(a => a.x), weightedIndex);       //smoothen the path
        tempCameraPath.y = doWMA(cameraPath.map(a => a.y), weightedIndex);      //smoothen the path
        //console.log(tempCameraPath);
        //need to add first elements separately due to MA filter losing them
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
        //cameraPath2 = cameraPath;     //comment for smoothing
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
        //nFrame = videoElement.webkitDecodedFrameCount + videoElement.webkitDroppedFrameCount - extraFrames;
        nFrame = Math.floor(videoElement.currentTime*fps);
        //console.log(nFrame);
        //console.log(prevFrame, nFrame);
        //let x = 0;
        //let y = 0;
        let dx = 0;
        let dy = 0;
        let trans = {"x": null, "y": null};
        //let delay = -10;
        var timeFromStart = null;
        //var cameraPos = null;
        let frameDataL = (nFrame-delay >=0 && nFrame-delay <= dataArray.length) ? dataArray[nFrame - delay] : dataArray[nFrame];
        //let frameDataL = dataArray[nFrame];
        if(nFrame === 0 && !videoElement.ended)
        {
                //console.log(dataArray);
                //timeAtStart = frameDataL.ori.time;
        }
        else if(nFrame !== 0 && nFrame !== prevFrame && frameDataL !== undefined && nFrame < videoElement.duration * fps)    //all subsequent frames
        {
                let frame = frameDataL.frame;
                //console.log(frame, nFrame);
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

