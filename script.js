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

var roll = null;
var pitch = null;
var yaw = null;
var accel = {"x": null, "y": null, "z": null};
var oriInitial = {"roll": null, "pitch": null, "yaw": null};
var initialoriobtained = false;

//canvas
var canvas = document.querySelector('canvas');
var ctx = canvas.getContext('2d');

//video element
var videoElement = document.querySelector('video');
videoElement.controls = false;

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
                if (this.onreading) this.onreading();
        };
        sensor.onactivate = () => {
                if (this.onactivate) this.onactivate();
        };
        const start = () => sensor.start();
        Object.assign(this, { start });
        }
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
                accel_sensor = new Accelerometer();
                accel_sensor.onreading = () => {
                        accel = {"x": accel_sensor.x, "y": accel_sensor.y, "z": accel_sensor.z};
                };
                accel_sensor.onactivate = () => {
                };
                accel_sensor.start();
                orientation_sensor = new AbsOriSensor();
                orientation_sensor.onreading = () => {
                        roll = orientation_sensor.roll;
                        pitch = orientation_sensor.pitch;
                        yaw = orientation_sensor.yaw;
                        if(!this.initialoriobtained) //obtain initial orientation
                        {
                                oriInitial = {"roll:": orientation_sensor.roll, "pitch:": orientation_sensor.pitch, "yaw:": orientation_sensor.yaw};
                                initialoriobtained = true;
                        }
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
	        videoElement.play();

	        mediaRecorder.ondataavailable = function(e) {
                        //console.log("Data available", e);
		        chunks.push(e.data);
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
                        
                        //resize canvas
videoElement.addEventListener('loadedmetadata', function() {
  canvas.width = videoElement.videoWidth;
  canvas.height = videoElement.videoHeight;
});

                        //Read blob data so we can stabilize the video                        
                        /*var reader = new FileReader();
                          reader.onload = function(event){
                                let text = reader.result;
                            console.log(text);
                          };
                        reader.readAsText(blob);*/
                        stabilize();
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
        //stabilize(blob);
	videoElement.controls = true;
}

function stabilize() {     //Idea: copy video to canvas, operate on the video, and then use the canvas with the stabilized video as source for the video element
        console.log(oriInitial);
        let x = 0;
        let y = 0;
        let width = 100;
        let height = 100;
        let oriDiff = {"roll": -oriInitial.roll, "pitch": -oriInitial.pitch, "yaw": -oriInitial.yaw};
        ctx.drawImage(videoElement,5,5);
        ctx.beginPath();
        ctx.rect(x,y,width,height);
        ctx.stroke();

        //ctx.drawImage(videoElement, 0, 0);
        let imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
        let pixeldataArray = [];
        //loop through every pixel
        for(let i=0; i<imageData.data.length; i = i+4)
        {
                let pixeldata = {"red": imageData.data[i], "green":imageData.data[i+1], "blue": imageData.data[i+2], "alpha":imageData.data[i+3]};
                pixeldataArray.push(pixeldata);
        }
        //console.log(pixeldataArray);
        //newImageData.data = data;
    // Draw the pixels onto the visible canvas
    //ctx.putImageData(newImageData,0,0);
        x = x + oriDiff.roll;
        y = y + oriDiff.pitch;

        requestAnimationFrame(stabilize);
}
