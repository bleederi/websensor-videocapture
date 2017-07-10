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

var constraints = {audio: true,video: {  width: { min: 640, ideal: 640, max: 640 },  height: { min: 480, ideal: 480, max: 480 }}};
var mediaRecorder = null;
var chunks = [];
var videoData = null;
var interval = null;

var accel_sensor = null;
var orientation_sensor = null;
var sensorfreq = 60;

//var roll = null;
//var pitch = null;
//var yaw = null;
var accel = {"x": null, "y": null, "z": null};
var ori = {"roll": null, "pitch": null, "yaw": null, "time": null};
var oriInitial = {"roll": null, "pitch": null, "yaw": null, "time": null};
var initialoriobtained = false;
var orientationData = [];       //array to store all the orientation data
var frameData = {"data": null, "time": null, "ori": null};
var dataArray = [];     //array to store all the combined data
var dataArray2 = [];     //array to store all the combined data

var time = null;
var timestamps = [];
var nFrame = 0; //frame number with which we can combine timestamp and frame data

//canvas
var canvas = document.querySelector('canvas');
var ctx = canvas.getContext('2d');

//video element
var videoElement = document.querySelector('video');
videoElement.controls = false;

var ref = null;

class AbsOriSensor {
        constructor() {
        const sensor = new AbsoluteOrientationSensor({ frequency: sensorfreq });
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

function startRecording(stream) {

                try {
                //Initialize sensors
                /*accel_sensor = new Accelerometer();
                accel_sensor.onreading = () => {
                        accel = {"x": accel_sensor.x, "y": accel_sensor.y, "z": accel_sensor.z};
                };
                accel_sensor.onactivate = () => {
                };
                accel_sensor.start();*/
                orientation_sensor = new AbsOriSensor();
                orientation_sensor.onreading = () => {
                        let roll = orientation_sensor.roll;
                        let pitch = orientation_sensor.pitch;
                        let yaw = orientation_sensor.yaw;
                        time = orientation_sensor.timestamp;
                        if(!initialoriobtained) //obtain initial orientation
                        {
                                oriInitial = {"roll": orientation_sensor.roll, "pitch": orientation_sensor.pitch, "yaw": orientation_sensor.yaw, "time": orientation_sensor.timestamp};
                                initialoriobtained = true;
                        }
                        ori = {"roll": orientation_sensor.roll, "pitch": orientation_sensor.pitch, "yaw": orientation_sensor.yaw, "time": orientation_sensor.timestamp};
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
	        //var options = {mimeType: 'video/webm;codecs=vp9'};
		//mediaRecorder = new MediaRecorder(stream, options);
		mediaRecorder = new MediaRecorder(stream);

	        mediaRecorder.start(10);

	        var url = window.URL;
	        videoElement.src = url ? url.createObjectURL(stream) : stream;	        
                //videoElement.play();

	        mediaRecorder.ondataavailable = function(e) {
                        //console.log("Data available", e);
                        //console.log(time);
                        timestamps.push(time);
                        frameData.time = time;
		        chunks.push(e.data);
                        frameData.data = e.data;         
                        orientationData.push(ori);
                        frameData.ori = ori;
                        var b = new Object;     //need to push by value
                        Object.assign(b, frameData);
                        dataArray.push(b);
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

		        var videoURL = window.URL.createObjectURL(blob);

		        videoElement.src = videoURL;
                        //videoElement.load();
                        
                        //resize canvas
videoElement.addEventListener('loadedmetadata', function() {
  canvas.width = videoElement.videoWidth;
  canvas.height = videoElement.videoHeight;
});

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
                        readFrameData(blob, orientationData);    //reads the video into dataArray2
                        /*videoElement.onended = function() {
                                alert("The video has ended");
                                cancelAnimationFrame(ref);
                        };*/
                        //ctx.clearRect(0, 0, canvas.width, canvas.height);
                        //stabilize(dataArray2);        //now we can operate on it
                        //interval=window.setInterval(stabilize,20);
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
function readFrameData(blob, oriArray) {     //Read video data from blob to object form with pixel data we can operate on
        //console.log(orientationData);
        //console.log(oriInitial);
        //console.log(timestamps[nFrame] - timestamps[0], videoElement.currentTime);
        //while(!videoElement.ended)
        //{
                let ori = orientationData[nFrame];
                let oriDiff = {"roll": ori.roll-oriInitial.roll, "pitch": ori.pitch-oriInitial.pitch, "yaw": ori.yaw-oriInitial.yaw};
                let x = 100*oriDiff.yaw;
                let y = 100*oriDiff.roll;
                let widthR = 100;
                let heightR = 100;
                ctx.drawImage(videoElement,0,0, videoElement.videoWidth, videoElement.videoHeight);
                ctx.beginPath();
                ctx.rect(x,y,widthR,heightR);
                ctx.stroke();

                //ctx.drawImage(videoElement, 0, 0);
                let imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
                let pixeldataArray = [];
                //loop through every pixel
                for(let i=0; i<imageData.data.length; i = i+4)
                {
                        let pixeldata = {"red": imageData.data[i], "green":imageData.data[i+1], "blue": imageData.data[i+2], "alpha":imageData.data[i+3]};
                        //pixeldataArray.push(pixeldata);
                }
                //pixeldataArray.push(imageData);
                //if(nFrame < dataArray.length) {
                        let timestamp = dataArray[nFrame].time;
                        let frameData2 = {"imagedata": imageData, "time": timestamp, "oridiff": oriDiff};
                        //var b = new Object;     //need to push by value
                        //Object.assign(b, frameData2);
                        //dataArray2.push(b);
                        dataArray2.push(frameData2);        
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
        if(nFrame < orientationData.length)
        {    
                nFrame = nFrame + 1;
                ref = requestAnimationFrame(readFrameData);
        }
}

function stabilize(dataArrayArg) { //Create a stabilized video from the pixel data given as input
        let frame = dataArrayArg[0];      //first frame
        console.log(frame);
        //ctx.drawImage(frame.imagedata,0,0, videoElement.videoWidth, videoElement.videoHeight);
        //ctx.putImageData(frame.imagedata, 0, 0);
}
