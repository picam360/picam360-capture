const fs = require('fs');
const BSON = require('bson');
const Long = BSON.Long;

var block_size = parseInt(process.argv[2]);
var input_path = process.argv[3];
var output_path = process.argv[4];
var config_json;

if(!(block_size > 0)){
	console.log('error:invalid block size:'+block_size);
	return;
}

console.log('check input_path:', input_path);
if (fs.existsSync(input_path + '/config.json')) {
	console.log('loading config...');
	config_json = JSON.parse(fs.readFileSync(input_path + '/config.json', 'utf8'));
	console.log('config_json:', config_json);
	config_json.block_size = block_size;
} else {
	console.log('error:invalid path');
	return;
}

console.log('check output_path:', output_path);
if (!fs.existsSync(output_path)) {
	console.log('create path...');
	fs.mkdirSync(output_path);
} else {
	console.log('error:already exists');
	return;
}

function bson_pack(ipaths, opath){
	//console.log(opath);
	//console.log(ipaths);
	var ary = [];
	for(var i=0;i<ipaths.length;i++){
		const data = fs.readFileSync(ipaths[i]);
		ary.push(data);
	}
	fs.writeFileSync(opath, BSON.serialize(ary));
}

fs.writeFileSync(output_path + '/config.json', JSON.stringify(config_json, null , '\t'));
for(var vp in config_json.keyframe_offset) {
	const ivp_path = input_path + '/' + vp;
	const ovp_path = output_path + '/' + vp;
	fs.mkdirSync(ovp_path);
	
	var ipaths = [];
	for(var i=0;;i++){
		const ipath = ivp_path + '/' + (i + 1) + '.pif';
		if(!fs.existsSync(ipath)){
			break;
		}
		ipaths.push(ipath);
	}
	const eblock = Math.ceil(ipaths.length/block_size);
	for(var i=0;i<eblock;i++){
		const opath = ovp_path + '/' + (i + 1) + '.bson';
		const sidx = i*block_size;
		const eidx = (i+1)*block_size;
		bson_pack(ipaths.slice(sidx, eidx), opath);
	}
	if(ipaths.length > 1){
		const opath = ovp_path + '/' + (eblock + 1) + '.bson';
		bson_pack(ipaths.slice(ipaths.length - 1), opath);
	}
}
