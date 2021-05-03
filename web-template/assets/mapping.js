var mapping = document.getElementById("mapping"),
    ctx_map = mapping.getContext("2d");
var i = 0;

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
    // In the first iteration there is nothing to restore
    if(i != 0){
        restoreDrawingSurface();
    }
    else{
        i = 1;
    }
    
	for(let d of dataLaser){
		ctx_map.fillRect(d[0],d[1],1,1);
	}
    saveDrawingSurface();
}
function saveDrawingSurface(){
    drawingSurfaceImageData = ctx_map.getImageData(0, 0, mapping.width, mapping.height);
}
function restoreDrawingSurface(){
    ctx_map.putImageData(drawingSurfaceImageData, 0, 0);
}