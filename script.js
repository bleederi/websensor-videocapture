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
var i = null;

var videoElement = document.querySelector('video');
videoElement.controls = false;

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
                accel_sensor.onchange = () => {
                };
                accel_sensor.onactivate = () => {
                };
                accel_sensor.start();
                orientation_sensor = new AbsOriSensor();
                orientation_sensor.onchange = () => {
                        this.roll = orientation_sensor.roll;
                        this.pitch = orientation_sensor.pitch;
                        this.yaw = orientation_sensor.yaw;
                        if(!this.initialoriobtained) //obtain initial longitude
                        {
                                let yawInitial = orientation_sensor.yaw;
                                this.longitudeInitial = -yawInitial * 180 / Math.PI;
                                longitudeOffset = this.longitudeInitial;
                                this.initialoriobtained = true;
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

                        //Read blob data so we can stabilize the video                        
                        /*var reader = new FileReader();
                          reader.onload = function(event){
                                let text = reader.result;
                            console.log(text);
                          };
                        reader.readAsText(blob);*/
                        stabilize(blob);
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

function stabilize(video) {     //Idea: copy video to canvas, operate on the video, and then use the canvas with the stabilized video as source for the video element
        var canvas = document.querySelector('canvas');
        var ctx = canvas.getContext('2d');
        i=window.setInterval(function() {ctx.drawImage(videoElement,5,5,260,125)},20);
        //ctx.drawImage(videoElement, 0, 0);
        //let imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
        //requestAnimationFrame(stabilize);
        //console.log(video);
}
