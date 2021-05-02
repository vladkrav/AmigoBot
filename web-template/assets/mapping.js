var mapping = document.getElementById("mapping"),
    ctx_map = mapping.getContext("2d");

var trail = [],
	coords = [-1, -1];;

function drawMapping(laser_data){
    mapping.width = 769;
    mapping.height = 729;
    drawMap(laser_data);
}
function clearMap(){
	ctx_map.clearRect(0, 0, mapping.width, mapping.height);
	trail = [];
}
function drawMap(dataLaser){
    ctx_map.fillStyle = "black";
	for(let d of dataLaser){
        // ctx_map.beginPath();
		ctx_map.fillRect(d[0],d[1],1,1);
        // ctx_map.closePath();
        ctx_map.save();
        
	}
}