// Set the src of iframe tag
//document.getElementById("gzweb").setAttribute(
//	"src", "http://" + websocket_address + ":8080")

// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s))
}

// Websocket and other variables for image display
var websocket_gui;
function declare_gui(websocket_address){
	websocket_gui = new WebSocket("ws://" + websocket_address + ":2303/");

	websocket_gui.onopen = function(event){
		alert("[open] Connection established!");
	}
	
	websocket_gui.onclose = function(event){
		if(event.wasClean){
			alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
		}
		else{
			//alert("[close] Connection closed!");
		}
	}

	// What to do when a message from server is received
	websocket_gui.onmessage = function(event){
		/*Extrae caracteres desde un indiceA hasta un indiceB sin incluirlo */
		var operation = event.data.substring(0, 4); /*Devuelve un subconunto de un objeto String*/
		
		if(operation == "#gui"){
			// Parse the entire Object
			/*Analiza una cadena de texto como JSON, transformando opcionalmente el valor producido por el analisis */
			var data = JSON.parse(event.data.substring(4, ));
			// var pose = data.map.substring(1, data.map.length - 1);
			// var content = pose.split(',').map(function(item) {
			// 	return parseFloat(item);
			// })
			var robot_coord = data.robot_coord;
			var robot_cont = data.robot_contorno;
			var laser_data = data.laser;
			var laser_global = data.laser_global;
			var sonar_sensor_point = data.sonar_sensor;
			var pos_vertices = data.pos_vertices;
			/*Draw all*/
			draw(robot_coord, robot_cont, laser_data, sonar_sensor_point, pos_vertices, laser_global);
			// Parse the Console messages
			messages = JSON.parse(data.text_buffer);
			// Loop through the messages and print them on the console
			for(message of messages){
				// Set value of command
				command.value = message
				// Go to next command line
				next_command()
			}


			// Send the Acknowledgment Message
			websocket_gui.send("#ack");
		}
		
		else if(operation == "#cor"){
			// Set the value of command
			var command_input = event.data.substring(4, );
			command.value = command_input;
			// Go to next command line
			next_command();
			// Focus on the next line
			command.focus();
		}
		
	}
}


// Image Display Configuration

// var canvas = document.getElementById("gui_canvas"),
//     context = canvas.getContext('2d');
//     image = new Image();
    
// Lap time DOM
// var lap_time_display = document.getElementById("lap_time");

// For image object
// image.onload = function(){
//     update_image();
// }

// Request Animation Frame to remove the flickers
// function update_image(){
// 	window.requestAnimationFrame(update_image);
// 	context.drawImage(image, 0, 0);
// }
