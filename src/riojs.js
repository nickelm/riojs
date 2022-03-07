/* --------------------------------------------------------------------
 * tomcatrio.js
 *
 * Browser game by Madgrim of VF-111.
 * Created February 2022. 
 * -------------------------------------------------------------------- 
 */

// -- Helper functions and classes

function toDegreesMinutesAndSeconds(coordinate) {
	var absolute = Math.abs(coordinate);
	var degrees = Math.floor(absolute);
	var minutesNotTruncated = (absolute - degrees) * 60;
	var minutes = Math.floor(minutesNotTruncated);
	var seconds = Math.floor((minutesNotTruncated - minutes) * 6);
	return degrees + "\u00B0" + minutes.toString().padStart(2, '0') + "'" + seconds + '"';
}

function convertDMS(lat, lng) {
	var latitude = toDegreesMinutesAndSeconds(lat);
	var latitudeCardinal = lat >= 0 ? "LN" : "LS";
	var longitude = toDegreesMinutesAndSeconds(lng);
	var longitudeCardinal = lng >= 0 ? "LE" : "LW";
	return latitudeCardinal + " " + latitude + "           " + longitudeCardinal + " " + longitude;
}

let deg2rad = (deg) => deg / 180.0 * Math.PI;
let rad2deg = (rad) => rad * 180.0 / Math.PI;
let kts2ms = (kts) => kts * 0.514444;
let ms2kts = (ms) =>ms / 0.514444;
let nm2m = (nm) => nm * 1852;
let m2nm = (m) => m / 1852;
let ft2m = (ft) => ft * 0.3048;
let m2ft = (m) => m / 0.3048;

function resize() {
	winWidth = window.innerWidth;
	winHeight = window.innerHeight - uiHeight;
	if (awg9) awg9.resize(winWidth, winHeight);
	app.renderer.resize(winWidth, winHeight);
}

class Vector3 {
	constructor(x = 0.0, y = 0.0, z = 0.0) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
	get length() {
		return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
	}
	add(v) {
		this.x += v.x;
		this.y += v.y;
		this.z += v.z;
	}
	scale(s) {
		this.x *= s;
		this.y *= s;
		this.z *= s;
	}
	normalize() {
		let len = this.length;
		this.x /= len;
		this.y /= len;
		this.z /= len;
	}
	getNormal() {
		let len = this.length;
		return new Vector3(this.x / len, this.y / len, this.z / len);
	}	
	getInverse() {
		return new Vector3(-this.x, -this.y, -this.z);
	}
}

class Vector2 {
	constructor(x = 0.0, y = 0.0) {
		this.x = x;
		this.y = y;
	}
	get length() {
		return Math.sqrt(this.x * this.x + this.y * this.y);
	}
	normalize() {
		let len = this.length;
		this.x /= len;
		this.y /= len;
	}
	dot(v) {
		return this.x * v.y + this.y * v.x;
	}
	add(v) {
		this.x += v.x;
		this.y += v.y;
	}
	scale(s) {
		this.x *= s;
		this.y *= s;
	}
	getNormal() {
		let len = this.length;
		return new Vector2(this.x / len, this.y / len);
	}
	getInverse() {
		return new Vector2(-this.x, -this.y);
	}
}

// FIXME: Horrible approximation!
function getLLfromXY(lat, lon, x, y) {
  const distPerLat = 40075017 / 360.0;
  const earthRadius = 6371000;
  let distPerLon = 2 * Math.PI * earthRadius * Math.cos(lat);
  return [ lat + y / distPerLat, lon + x / distPerLon ];
}

function drawDashedLine(g, x1, y1, x2, y2, dashLen) {
	let vec = new Vector2(x2 - x1, y2 - y1);
	let dist = vec.length;
	vec.normalize();
	let currDist = 0.0;
	while (currDist < dist) {
		g.moveTo(x1 + currDist * vec.x, y1 + currDist * vec.y);
		currDist += dashLen;
		if (currDist > dist) currDist = dist;
		g.lineTo(x1 + currDist * vec.x, y1 + currDist * vec.y);
		currDist += dashLen;
	}
}

class PIDController {

	constructor(Kp, Ki, Kd, maxOutput, maxChange) {
		this.target = 0.0;
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		this.integral = 0.0;
		this.output = 0.0;
		this.preError = 0.0;
		this.maxOutput = maxOutput;
		this.maxChange = maxChange;
		this.outputLast = 0.0;
	}

	setTarget(target) {
		this.target = target;
	}

	getTarget() {
		return this.target;
	}

	update(delta, pv) {

		// Calculate the output
		let error = this.target - pv;
		this.integral = this.integral + (error * delta);
		this.derivative = (error - this.preError) / delta;
		this.output = (this.Kp * error) + (this.Ki * this.integral) + (this.Kd * this.derivative);
		this.preError = error;

		// Enforce limits
		if (this.output > this.maxOutput) this.output = this.maxOutput;
		if (this.output < -this.maxOutput) this.output = -this.maxOutput;
		if ((this.output - this.outputLast) > this.maxChange) this.output = this.outputLast + this.maxChange;
		if ((this.outputLast - this.output) > this.maxChange) this.output = this.outputLast - this.maxChange;
		this.outputLast = this.output;
		
		return this.output;
	}
}

let flightModel = [
	{ throttle: 0.55, speed: kts2ms(150) },
	{ throttle: 0.60, speed: kts2ms(200) },
	{ throttle: 0.70, speed: kts2ms(300) },
	{ throttle: 0.80, speed: kts2ms(400) },
	{ throttle: 0.90, speed: kts2ms(500) },
	{ throttle: 1.00, speed: kts2ms(600) },
	{ throttle: 1.10, speed: kts2ms(800) },
	{ throttle: 1.20, speed: kts2ms(1100) },
];

function wrapdeg(deg) {
	while (deg < 0) deg += 360.0;
	while (deg >= 360) deg -= 360.0;
	return deg;
}

function toaspect(deg) {
	aspect = wrapdeg(deg);
	if (aspect > 180.0) aspect -= 360.0;
	return aspect;
}

function aspect2str(deg) {
	deg = Math.abs(deg);
	if (deg < 30) return "hot";
	if (deg < 60) return "flanking";
	if (deg < 120) return "beaming";
	return "cold";
}

function lerp(a, b, t) {
	return a * (1 - t) + b * t;
}

function getVelocity(heading, pitch, speed) {
	let hdg = deg2rad(heading + 90.0);
	let pitch_rad = deg2rad(pitch);
	return new Vector3(
		-speed * Math.cos(hdg) * Math.cos(pitch_rad),
		 speed * Math.sin(hdg) * Math.cos(pitch_rad),
		 speed * Math.sin(pitch_rad));
}

function calculateAcceleration(throttle, speed, pitch, delta) {

	// Look up the right position
	let i = 1;
	while (i < flightModel.length - 1 && flightModel[i].throttle < throttle) i++;

	// Interpolate the actual speed
	let targetSpeed = lerp(flightModel[i - 1].speed, flightModel[i].speed, (throttle - flightModel[i - 1].throttle) / (flightModel[i].throttle - flightModel[i - 1].throttle));

	// Now calculate the acceleration
	let acc = 0.01 * (targetSpeed - speed);
//	if (acc > 0.5) acc = 0.5;
	//if (acc < -0.5) acc = -0.5;

	// Adjust acceleration based on pitch
	acc -= pitch / 20.0;
	
	// Add some noise
	acc += (Math.random() - 0.5) * 0.1;

	return acc;
}

function rot2d(v, deg) {
	let xd = v.x * Math.cos(deg2rad(deg)) - v.y * Math.sin(deg2rad(deg));
	let yd = -v.y * Math.cos(deg2rad(deg)) - v.x * Math.sin(deg2rad(deg));
	return new Vector2(xd, yd);
}

function braa(pos1, vel1, pos2, vel2) {
	let diff = new Vector2(pos2.x - pos1.x, pos2.y - pos1.y);
	let diff3 = new Vector3(pos2.x - pos1.x, pos2.y - pos1.y, pos2.z - pos1.z);
	let bearing = Math.atan2(diff.x, diff.y);
	let fh = Math.atan2(vel1.x, vel1.y);
	let bh = Math.atan2(vel2.x, vel2.y);
	let br = bh + Math.PI;
	let fclosure = vel1.length * Math.cos(bearing - fh); 
	let bclosure = vel2.length * Math.cos(bearing - bh);
	let va = Math.atan2(pos2.z - pos1.z, diff.length);

	return {
		brg: wrapdeg(rad2deg(bearing)),
		rng: diff.length,
		sr: diff3.length,
		alt: pos2.z,
		fh: wrapdeg(rad2deg(fh)),
		ata: toaspect(rad2deg(bearing - fh)),
		bh: wrapdeg(rad2deg(bh)),
		bs: vel2.length,
		ta: toaspect(rad2deg(bearing - br)),
		closure: fclosure - bclosure,
		va: toaspect(rad2deg(va)),
		ls: diff.length * Math.sin(bearing - br)
	};
}

function getRandom(min, max) {
	let diff = max - min;
	return diff * Math.random() + min;
}

class Bogey {
	constructor(x, y, heading, altitude, speed) {
		this.pos = new Vector3(x, y, altitude);
		this.speed = speed;
		this.heading = heading;
		this.pitch = 0.0;
		this.type = 'unknown';
		this.velocity = getVelocity(this.heading, this.pitch, this.speed);
	}
	update(delta) {
		this.velocity = getVelocity(this.heading, this.pitch, this.speed);
		this.pos.x += delta * this.velocity.x;
		this.pos.y += delta * this.velocity.y;
		this.pos.z += delta * this.velocity.z;
	}
}

class World {

	constructor() {
		this.bogeys = [];
	}

	createBogey(brg, hdg, distance, alt, speed) {
		let x = tomcat.pos.x - distance * Math.cos(deg2rad(brg + 90));
		let y = tomcat.pos.y + distance * Math.sin(deg2rad(brg + 90));
		let bogey = new Bogey(x, y, hdg, alt, speed);
		this.bogeys.push(bogey);
		return bogey;
	}

	update(delta) {
		for (const bogey of this.bogeys) {
			bogey.update(delta);
		}
	}

	clear() {
		for (const bogey of this.bogeys) {
			if (bogey.contact) {
				awg9.display.removeChild(bogey.contact.gfx);
			}
			if (awg9.hooked == bogey) awg9.hooked = null;
		}
		this.bogeys = [];
	}
}

class Tomcat {

	constructor(lat, lon, heading, altitude, speed) {
		this.lat = lat;
		this.lon = lon;
		this.heading = heading;
		this.speed = speed;
		this.pitch = 0.0;
		this.roll = 0.0;
		this.throttle = 0.80;
		this.pos = new Vector3(0, 0, altitude);
		this.speedPID = new PIDController(0.0001, 0, 0, 0.01, 0.001);
		this.speedPID.setTarget(speed);
		this.altPID = new PIDController(1.0, 0, 0.15, 20, 1.0);
		this.altPID.setTarget(altitude);
		this.rollPID = new PIDController(0.1, 0, 0, 5, 0.1);
		this.rollPID.setTarget(0.0);
		this.pidDelta = 0.0;
		this.currTurn = 'steady';
		this.currPitch = 'hold';
		this.velocity = getVelocity(this.heading, this.pitch, this.speed);
	}

	setAltitude(altitude) {
		this.altPID.setTarget(altitude);
	}

	setRoll(roll) {
		this.rollPID.setTarget(roll);
	}

	getRoll() {
		return this.rollPID.getTarget();
	}

	setSpeed(speed) {
		this.speedPID.setTarget(speed);
	}

	update(delta) {

		// Don't update the controller too often (instability)
		this.pidDelta += delta;
		if (this.pidDelta > 3 * delta) {

			// Manage altitude
			this.pitch = this.altPID.update(this.pidDelta, this.pos.z);

			// Update speed
			this.throttle += this.speedPID.update(this.pidDelta, this.speed);
			if (this.throttle > 1.2) this.throttle = 1.2;
			if (this.throttle < 0.6) this.throttle = 0.6;

			// Update the roll and heading
			this.roll += this.rollPID.update(this.pidDelta, this.roll);
	
			// Reset update timer
			this.pidDelta = 0.0;
		}

		// Speed update
		this.speed += calculateAcceleration(this.throttle, this.speed, this.pitch, delta);

		// Update heading based on angle of roll
		this.heading += delta * 1091 / ms2kts(this.speed) * Math.tan(deg2rad(this.roll));
		while (this.heading < 0) this.heading += 360.0;
		while (this.heading >= 360.0) this.heading -= 360.0;
		
		// Build velocity vector
		this.velocity = getVelocity(this.heading, this.pitch, this.speed);

		// Update position
		this.pos.x += delta * this.velocity.x;
		this.pos.y += delta * this.velocity.y;
		this.pos.z += delta * this.velocity.z;

		// Update the status display
		$('#status').html(`HDG: <b>${Math.round(this.heading).toString().padStart(3, '0')}</b> | IAS: <b>${Math.floor(ms2kts(this.speed))}</b> | ALT: <b>${Math.floor(m2ft(this.pos.z))}</b><br/>
			Roll: <b>${Math.abs(this.roll) < 0.5 ? "steady" : (this.roll < 0 ? "left " : "right ") + Math.abs(Math.round(this.roll)) + "&deg;"}</b> |
			Throttle: <b>${Math.round(100 * this.throttle)+"%"}</b> |
			Pitch: <b>${Math.abs(this.pitch) < 0.5 ? "steady" : (this.pitch < 0 ? "down " : "up ") + Math.abs(Math.round(this.pitch)) + "&deg;"}</b><br/>`);
	}
}

class Contact {

	constructor(bogey, size) {
		this.bogey = bogey;
		this.size = size;
		this.gfx = new PIXI.Graphics();
		this.gfx.interactive = true;
		this.gfx.hitArea = new PIXI.Rectangle(-this.size / 2, -this.size / 2, this.size, this.size);
		this.gfx.click = () => {
			awg9.hooked = this.bogey;
			let data = braa(tomcat.pos, tomcat.velocity, this.bogey.pos, this.bogey.velocity);
			messages.add(`RIO: New radar contact, BRAA ${Math.round(data.brg)}, ${Math.round(m2nm(data.sr))} miles, ${Math.round(m2ft(data.alt) / 1000.0) * 1000} feet, ${aspect2str(data.ta)}.`, 5.0);
		};
		this.alt = new PIXI.Text(Math.ceil(m2ft(this.bogey.pos.z) / 10000), awg9Style);
		this.alt.anchor.set(1, 0.5);
		this.alt.x = -this.size / 2;
		this.alt.y = 0;
		this.gfx.addChild(this.alt);
	}

	update(x, y, xd, yd) {
		this.gfx.x = x;
		this.gfx.y = y;
		this.gfx.clear();
		if (awg9.hooked === this.bogey) {
			this.gfx.lineStyle(3, awg9Selection);
			this.alt.style.stroke = awg9Selection;
		}
		else {
			this.gfx.lineStyle(2, awg9Color);
			this.alt.style.stroke = awg9Color;
		}
		this.gfx.drawCircle(0, 0, 1)
		this.gfx.moveTo(-this.size / 2, 0);
		this.gfx.lineTo(-this.size / 2, -this.size / 2);
		this.gfx.lineTo(this.size / 2, -this.size / 2);
		this.gfx.lineTo(this.size / 2, 0);
		this.gfx.moveTo(0, 0);
		this.gfx.lineTo(xd, yd);
		this.alt.text = Math.ceil(m2ft(this.bogey.pos.z) / 10000);
	}
}

class AWG9 {

	constructor(tomcat, width, height) {
		this.tomcat = tomcat;
		this.azimuth = 0;
		this.elevation = 0;
		this.azScan = 40;
		this.bars = 4;
		this.range = 100;
		this.buildDisplay(width, height);
		this.scanHeight = { 1: 2.3, 2: 3.6, 4: 6.3, 8: 11.5 };
		this.hooked = null;
		this.tidMode = 'ac-stab';
	}

	resize(width, height) {
		this.bg.width = width;
		this.bg.height = height;
	}

	buildDisplay(width, height) {

		this.display = new PIXI.Container();

		this.bg = new PIXI.Sprite(PIXI.Texture.BLACK);
		this.bg.width = width;
		this.bg.height = height;
		this.bg.interactive = true;
		this.bg.click = () => { this.hooked = null; }
		this.display.addChild(this.bg);

		this.scan = new PIXI.Graphics();
		this.display.addChild(this.scan);

		this.elevLabels = new PIXI.Text('50\n-0.00\n0', awg9Style);
		this.statusLabels = new PIXI.Text('LN 0\u00B000"0  LE 0\u00B000"0', awg9Style);
		this.statusLabels.anchor.set(0.5, 0);
		this.closureLabel = new PIXI.Text('+0', awg9Style);
		this.closureLabel.anchor.set(1.0, 0);
		this.display.addChild(this.elevLabels);
		this.display.addChild(this.statusLabels);
		this.display.addChild(this.closureLabel);
	}

	insideRadarVolume(data) {
				
		// Inside the azimuth scan?
		if (data.ata < this.azimuth - this.azScan || data.ata > this.azimuth + this.azScan) return false;

		// Inside the vertical scan volume?
		if (data.va < this.elevation - this.scanHeight[this.bars] / 2 ||
			data.va > this.elevation + this.scanHeight[this.bars] / 2) return false;

		// Is it inside the range?
		// FIXME: Model size of target
		if (data.rng > nm2m(200)) return false;

		return true;
	}

	updateContacts(delta) {

		// Step through all bogeys
		for (const bogey of world.bogeys) {

			// Calculate the BRAA
			let data = braa(tomcat.pos, tomcat.velocity, bogey.pos, bogey.velocity);

			// Is this bogey inside the radar volume?
			if (!this.insideRadarVolume(data)) {				
				if (bogey.contact) {
					this.display.removeChild(bogey.contact.gfx);
					if (this.hooked == bogey) this.hooked = null;
				}
				continue;
			}

			// Create the contact if needed
			if (bogey.contact === undefined) {
				bogey.contact = new Contact(bogey, 25);
			}
			
			// Add it to the container if needed
			if (!this.display.children.includes(bogey.contact.gfx)) {
				this.display.addChild(bogey.contact.gfx);
			}

			// Draw the contact
			if (this.tidMode === 'gnd-stab') {

				// Calculate coordinates
				let displayDim = Math.min(winWidth, winHeight);
				let nmPerPixel = displayDim / this.range;
				let xp = winWidth / 2 - m2nm(bogey.pos.x - tomcat.pos.x) * nmPerPixel;
				let yp = winHeight / 2 - m2nm(bogey.pos.y - tomcat.pos.y) * nmPerPixel;

				// Calculate closure vector
				let xd = ms2kts(bogey.velocity.x) / nmPerPixel;
				let yd = -ms2kts(bogey.velocity.y) / nmPerPixel;

				// Update the shape
				bogey.contact.update(xp, yp, xd, yd);
			}
			else {
			
				// Calculate coordinates
				let displayHeight = winHeight - 3 * tidScale;
				let nmPerPixel = displayHeight / this.range;
				let pixelDist = m2nm(data.rng) * nmPerPixel;
				let xp = winWidth / 2 + pixelDist * Math.sin(deg2rad(data.ata));
				let yp = displayHeight - pixelDist * Math.cos(deg2rad(data.ata));

				// Calculate closure vector
				let vt = tomcat.velocity.getInverse();
				vt.add(bogey.velocity);
				vt.scale(1 / nmPerPixel); // m/s?
				let rd = rot2d(vt, tomcat.heading);

				// Update the shape
				bogey.contact.update(xp, yp, rd.x, rd.y);
			}
		}
	}

	update(delta) {

		// Check the antenna angle
		if (this.elevation < -80) this.elevation = -80;
		if (this.elevation > 60) this.elevation = 60;
		if (this.azimuth + this.azScan > 65) this.azimuth = 65 - this.azScan;
		if (this.azimuth - this.azScan < -65) this.azimuth = this.azScan - 65;

		// Update the radar scan volume
		this.scan.clear();
		this.scan.lineStyle(2, awg9Color);
		let diffY = winHeight - 3 * tidScale;

		// Are we drawing a ground-stabilized radar picture?
		if (this.tidMode === 'gnd-stab') {
			
			// Radar arc line is the diagonal to the corner
			let lineLen = new Vector2(winWidth / 2, winHeight / 2).length;

			// Dash length
			let displayDim = Math.min(winWidth, winHeight);
			let nmPerPixel = displayDim / this.range;
			let dashLen = nmPerPixel * 20;
			
			// Calculate two points
			let v1 = new Vector2(lineLen * Math.sin(deg2rad(-this.azScan)), lineLen * Math.cos(deg2rad(-this.azScan)));
			let v2 = new Vector2(lineLen * Math.sin(deg2rad(this.azScan)), lineLen * Math.cos(deg2rad(this.azScan)));

			// Rotate the points
			let p1 = rot2d(v1, -(tomcat.heading + this.azimuth));
			let p2 = rot2d(v2, -(tomcat.heading + this.azimuth));

			drawDashedLine(this.scan, winWidth / 2, winHeight / 2, winWidth / 2 + p1.x, winHeight / 2 + p1.y, dashLen);
			drawDashedLine(this.scan, winWidth / 2, winHeight / 2, winWidth / 2 + p2.x, winHeight / 2 + p2.y, dashLen);

			// Draw own aircraft
			this.scan.lineStyle(1, awg9Color);
			this.scan.drawCircle(winWidth / 2, winHeight / 2, tidScale);

			// Draw own direction vector
			let xd = ms2kts(tomcat.velocity.x) / nmPerPixel;
			let yd = -ms2kts(tomcat.velocity.y) / nmPerPixel;
			this.scan.moveTo(winWidth / 2, winHeight / 2);
			this.scan.lineTo(winWidth / 2 + xd, winHeight / 2 + yd);
		}
		// No, it's aircraft-stabilized
		else {
			let diffLeft = Math.tan(deg2rad(this.azScan - this.azimuth)) * diffY;
			let diffRight = Math.tan(deg2rad(this.azScan + this.azimuth)) * diffY;
			let dashLen = diffY / this.range * 20;
			drawDashedLine(this.scan, winWidth / 2, diffY, winWidth / 2 - diffLeft, 0, dashLen);
			drawDashedLine(this.scan, winWidth / 2, diffY, winWidth / 2 + diffRight, 0, dashLen);

			// Draw own aircraft
			this.scan.lineStyle(1, awg9Color);
			this.scan.drawCircle(winWidth / 2, diffY, tidScale);
		}

		// Now draw the horizon line
		if (this.tidMode === 'attk') {
			this.scan.lineStyle(2, awg9Color);
			let horzY = diffY / 2 * Math.sin(deg2rad(tomcat.pitch)) + diffY / 2;
			let bankY = winWidth / 2 * Math.tan(deg2rad(tomcat.roll));
			//drawDashedLine(this.scan, 0, horzY + bankY, winWidth, horzY - bankY, winWidth / 3.0);
			this.scan.moveTo(0, horzY + bankY);
			this.scan.lineTo(winWidth, horzY - bankY, winWidth / 3.0);
		}

		// Check the contacts
		this.updateContacts(delta);

		// Calculate vertical scan volume (from: https://forums.eagle.ru/topic/244080-awg-9-scan-pattern-diagrams/)
		let b = nm2m(this.range);
		let a = b * Math.tan(deg2rad(this.elevation));
		let coneHeight = b * Math.tan(deg2rad(this.scanHeight[this.bars] / 2));
		let Hlim = Math.round(m2ft(this.tomcat.pos.z + a + coneHeight) / 1000);
		let Llim = Math.round(m2ft(this.tomcat.pos.z + a - coneHeight) / 1000);

		// Now update the elevation information
		this.elevLabels.text = '' + Hlim + '\n' + this.elevation.toFixed(2) + '\n' + Llim;
		this.elevLabels.x = tidScale;
		this.elevLabels.y = diffY / 2;
		this.closureLabel.x = winWidth - tidScale;
		this.closureLabel.y = diffY / 2;

		// Update the status
		if (this.hooked == null) {
			let pos = getLLfromXY(tomcat.lat, tomcat.lon, tomcat.pos.x, tomcat.pos.y);
			this.statusLabels.text = convertDMS(pos[0], pos[1]);
			this.closureLabel.text = '+0';
		}
		else {
			let data = braa(tomcat.pos, tomcat.velocity, this.hooked.pos, this.hooked.velocity);
			this.statusLabels.text = `${data.ta < 0 ? 'LT' : 'RT'} ${Math.round(Math.abs(data.ta))}    MC ${Math.round(data.bh)}   BR ${ data.ata < 0 ? Math.round(data.ata + 360) : Math.round(data.ata)}
			RA ${Math.round(m2nm(data.sr))}   AS ${Math.round(ms2kts(data.bs))}   AL +${Math.round(m2ft(data.alt))}`;
			this.closureLabel.text = data.closure > 0 ? '+' + Math.round(ms2kts(data.closure)) : Math.round(ms2kts(data.closure));
		}
		this.statusLabels.x = winWidth / 2;
		this.statusLabels.y = tidScale * 2;
	}
}

function setRange(e) {
	$('#r' + awg9.range).removeClass('pressed');
	awg9.range = parseInt(e.target.innerHTML);
	$('#r' + awg9.range).addClass('pressed'); 
}

function setAZScan(e) {
	$('#az' + awg9.azScan).removeClass('pressed');
	awg9.azScan = parseInt(e.target.innerHTML);
	$('#az' + awg9.azScan).addClass('pressed');   
}

function setELBars(e) {
	$('#bars' + awg9.bars).removeClass('pressed');
	awg9.bars = parseInt(e.target.innerHTML);
	$('#bars' + awg9.bars).addClass('pressed');   
}

function setTIDMode(e) {
	$('#' + awg9.tidMode).removeClass('pressed');
	awg9.tidMode = e.target.id;
	$('#' + awg9.tidMode).addClass('pressed');
}

let fireInterval = null;

function elevate(delta) {
  awg9.elevation += delta;
}

function turnRadar(delta) {
  awg9.azimuth += delta;
}

let rollCommands = [ 'left-break', 'left-hard', 'left-std', 'steady', 'right-std', 'right-hard', 'right-break' ];
let rollValues = { 'left-break': -60, 'left-hard': -45, 'left-std': -30, 'steady': 0, 'right-std': 30, 'right-hard': 45, 'right-break': 60 };
let rollMessages = { 'left-break': 'break left!', 'left-hard': 'left hard.', 'left-std': 'left standard.', 'steady': 'steady.', 'right-std':'right standard.', 'right-hard':'right hard.', 'right-break':'break right!' };

function rollTomcat(id) {
	if (tomcat.currTurn != '') $('#' + tomcat.currTurn).removeClass('pressed');
	tomcat.setRoll(rollValues[id]);
	tomcat.currTurn = id;
	$('#' + tomcat.currTurn).addClass('pressed');
	messages.add('RIO: ' + rollMessages[id]);
}

function changeTomcatRoll(delta) {
	if (tomcat.currTurn == '') return;
	let curr = rollCommands.indexOf(tomcat.currTurn);
	curr += delta;
	if (curr < 0 || curr >= rollCommands.length) return;
	rollTomcat(rollCommands[curr]);
}

function setTomcatAltitude(alt, id, message) {
	if (tomcat.currPitch != '') $('#' + tomcat.currPitch).removeClass('pressed');
	tomcat.setAltitude(alt);
	tomcat.currPitch = id;
	$('#' + tomcat.currPitch).addClass('pressed');
	messages.add('RIO: ' + message);
}


let zeroCutTutorial = {
	title: "Tutorial: Zero Cut Intercept",
	init: () => {
		let brg = Math.random() * 360;
		let hdg = wrapdeg(brg + 180 + getRandom(-5, 5));
		let alt = getRandom(10000, 40000);
		let spd = getRandom(250, 600);
		let range = 100;
		let bogey = world.createBogey(brg, hdg, nm2m(range), ft2m(alt), kts2ms(spd));
		messages.add(`${callsignAWACS.toUpperCase()}: ${callsignPlayer}, ${callsignAWACS}, new group, BRAA ${Math.round(brg)}, ${range} miles, ${Math.round(alt / 1000.0) * 1000} feet, hot.`, 5.0);
		return bogey;
	},
	sequence: [
		[
			{ task: (t) => 'Turn to bogey bearing ' + Math.round(t.brg), check: (t) => Math.abs(t.ata) < 10.0 },
			{ task: (t) => 'Roll out on bearing', check: (t) => Math.abs(tomcat.roll) < 10.0, conditional: true }
		],
		[ 
			{ task: (t) => 'Match bogey altitude ' + Math.round(m2ft(t.alt) / 1000.0) * 1000, check: (t) => Math.abs(t.alt - tomcat.pos.z) < 500 },
			{ task: (t) => 'Hook (select) the target', check: (t) => awg9.hooked !== null },
			{ task: (t) => 'Match reciprocal bogey heading ' + Math.round(wrapdeg(t.bh - 180)), check: (t) => Math.abs(t.fh - (t.bh - 180)) < 10 },
			{ task: (t) => 'Increase speed to 500+ for high-speed intercept', check: (t) => tomcat.speed >= kts2ms(490) },
		],
		[
			{ task: (t) => 'Maintain TA of 5&#177;2&deg; at 50 nm', check: (t) => Math.abs(Math.abs(t.ta) - 5) < 2 },
			{ task: (t) => 'Approach to 50 nm', check: (t) => m2nm(t.sr) < 50, conditional: true },
		],
		[
			{ task: (t) => 'Maintain TA of 10&#177;2&deg; at 40 nm', check: (t) => Math.abs(Math.abs(t.ta) - 10) < 2 },
			{ task: (t) => 'Approach to 40 nm', check: (t) => m2nm(t.sr) < 40, conditional: true },
		],
		[
			{ task: (t) => 'Maintain TA of 15&#177;2&deg; at 30 nm', check: (t) => Math.abs(Math.abs(t.ta) - 15) < 2 },
			{ task: (t) => 'Approach to 30 nm', check: (t) => m2nm(t.sr) < 30, conditional: true },
		],
		[
			{ task: (t) => 'Maintain TA of 20&#177;2&deg; at 20 nm', check: (t) => Math.abs(Math.abs(t.ta) - 20) < 2 },
			{ task: (t) => 'Approach to 20 nm', check: (t) => m2nm(t.sr) < 20, conditional: true },
		],
		[
			{ task: (t) => 'Maintain TA of 30&#177;2&deg; at 15 nm', check: (t) => Math.abs(Math.abs(t.ta) - 30) < 2 },
			{ task: (t) => 'Approach to 15 nm', check: (t) => m2nm(t.sr) < 15, conditional: true },
		],
		[
			{ task: (t) => 'Maintain TA of 40&#177;2&deg; at 10 nm', check: (t) => Math.abs(Math.abs(t.ta) - 40) < 2 },
			{ task: (t) => 'Approach to 10 nm', check: (t) => m2nm(t.sr) < 10, conditional: true },
		],
		[
			{ task: (t) => 'Go pure pursuit (point nose on target)', check: (t) => Math.abs(t.ata) < 5.0 },
			{ task: (t) => 'Reduce closure to &lt;50 kts', check: (t) => Math.abs(t.closure) < ms2kts(50), conditional: true },
			{ task: (t) => 'Approach within 2 nm', check: (t) => m2nm(t.sr) < 2, conditional: true },
		],
		[{ task: (t) => 'Success!', check: (t) => false }]
	]
};

let kickAndBuildTutorial = {
	title: "Tutorial: Kick-and-Build Intercept",
	init: () => {
		let brg = Math.random() * 360;
		let hdg = wrapdeg(brg + 180 + getRandom(10, 20));
		let alt = getRandom(10000, 40000);
		let spd = getRandom(250, 600);
		let range = 100;
		let bogey = world.createBogey(brg, hdg, nm2m(range), ft2m(alt), kts2ms(spd));
		messages.add(`${callsignAWACS.toUpperCase()}: ${callsignPlayer}, ${callsignAWACS}, new group, BRAA ${Math.round(brg)}, ${range} miles, ${Math.round(alt / 1000.0) * 1000} feet, hot.`, 5.0);
		return bogey;
	},
	sequence: [
		[
			{ task: (t) => 'Turn to bogey bearing ' + Math.round(t.brg), check: (t) => Math.abs(t.ata) < 10.0 },
			{ task: (t) => 'Roll out on bearing', check: (t) => Math.abs(tomcat.roll) < 10.0, conditional: true }
		],
		[ 
			{ task: (t) => 'Hook (select) the target', check: (t) => awg9.hooked !== null },
			{ task: (t) => 'Match bogey altitude ' + Math.round(m2ft(t.alt) / 1000.0) * 1000, check: (t) => Math.abs(t.alt - tomcat.pos.z) < 500, conditional: true},
			{ task: (t) => 'Match bogey speed', check: (t) => Math.abs(tomcat.velocity.length - t.bs) < kts2ms(50)},
		],
		[
			{ task: (t) => 'Turn right until bogey at radar gimbal limit' , check: (t) => Math.abs(Math.abs(t.ata) - 60) < 10 },
			{ task: (t) => 'Achieve TA of 40&#177;2&deg;' , check: (t) => Math.abs(Math.abs(t.ta) - 40) < 2, conditional: true},
		],
		[
			{ task: (t) => 'Maintain TA of 40&#177;2&deg;' , check: (t) => Math.abs(t.ta + 40) < 2 },
			{ task: (t) => 'Set collision course (ATA/bearing 40&#177;2&deg;)' , check: (t) => Math.abs(t.ata - 40) < 2 },
			{ task: (t) => 'Approach to 10 nm', check: (t) => m2nm(t.sr) < 10, conditional: true },
		],
		[
			{ task: (t) => 'Go pure pursuit (point nose on target)', check: (t) => Math.abs(t.ata) < 5.0 },
			{ task: (t) => 'Reduce closure to &lt;50 kts', check: (t) => Math.abs(t.closure) < ms2kts(50), conditional: true },
			{ task: (t) => 'Approach within 2 nm', check: (t) => m2nm(t.sr) < 2, conditional: true },
		],
		[{ task: (t) => 'Success!', check: (t) => false }]
	]
};

class Scenario {
	constructor(title) {
		this.title = title;
	}
	init() {}
	update(delta) {}
	cleanup() {}
}

class TutorialScenario extends Scenario {
	constructor(tutorial) {
		super(tutorial.title);
		this.tutorial = tutorial;
	}
	init() {
		this.bogey = this.tutorial.init();
		this.pos = 0;
		this.timeOut = 0.0;
	}
	update(delta) {

		// Are we in timeout? If so, don't change the display
		if (this.timeOut > 0.0) {
			this.timeOut -= delta;
			return;
		}
		$('#scenario-status').removeClass('success');

		// Check the conditions
		let t = braa(tomcat.pos, tomcat.velocity, this.bogey.pos, this.bogey.velocity);
		let msg = 'Current scenario: <b>' + scenario.title + '</b><br/>';
		let allSat = true;
		for (let cond of this.tutorial.sequence[this.pos]) {
			if (cond.conditional === true && !allSat) break;
			let currSat = cond.check(t);
			allSat = allSat && currSat;
			msg += '<i class="fa-solid ' + (currSat ? 'fa-check' : 'fa-xmark') + '"></i> <i>' + cond.task(t) + "</i><br/>";
		}
		$('#scenario-status').html(msg);

		// Are all of the conditions fulfilled?
		if (allSat && this.pos < this.tutorial.sequence.length - 1) {
			this.pos += 1;
			this.timeOut = 1.5;
			$('#scenario-status').addClass('success');
		}
	}
	cleanup() {
		world.clear();
	}
}

class InterceptScenario extends Scenario {
	constructor(range) {
		super("Interception " + range + "nm");
		this.range = range;
	}
	init() {
		let brg = Math.random() * 360;
		let hdg = brg + 180 + getRandom(-45, 45);
		let alt = getRandom(5000, 40000);
		let spd = getRandom(200, 600);
		world.createBogey(brg, hdg, nm2m(this.range), ft2m(alt), kts2ms(spd));
		messages.add(`${callsignAWACS.toUpperCase()}: ${callsignPlayer}, ${callsignAWACS}, new group, BRAA ${Math.round(brg)}, ${this.range} miles, ${Math.round(alt / 10000.0) * 10000} feet, hot.`, 5.0);
	}
	update(delta) {}
	cleanup() {
		world.clear();
	}
}

class MessageSystem {
	constructor() {
		this.elapsed = 0.0;
		this.buffer = [];
		this.text = new PIXI.Text("OVERLORD: Hello", messageStyle);
		this.text.anchor.set(0.5, 0);
		this.currTime = 0.0;
	}
	add(text, duration = 1.5) {
		this.buffer.push({ text: text, expire: this.currTime + duration, time: Date.now() });
		$('#history').append(text + '\n');
	}
	update(delta) {
		
		// Update position
		this.text.x = winWidth / 2;
		this.text.y = 96.0;

		// Update time
		this.currTime += delta;

		// Rebuild the string
		let text = [];
		let expired = [];
		for (const msg of this.buffer) {

			// is this message still live?
			if (msg.expire > this.currTime) {
				text = msg.text + "\n" + text;
			}
		}

		// Set the string
		this.text.text = text;
	}
}

// -- Constants
const uiHeight = 128;
const tidScale = 12;
const awg9Color = 0x00ff00;
const awg9Selection = 0xaaffaa;
const awg9Style = new PIXI.TextStyle({
	fontFamily: 'Tahoma',
	fontSize: 24,
	fill: awg9Color,
	align: 'center'
});

const callsignAWACS = 'Overlord';
const callsignPlayer = 'Sundown 1-1';

const messageStyle = new PIXI.TextStyle({
	fontFamily: 'Arial',
	fontSize: 36,
	fill: ['#ffffff', '#00ff99'],
    stroke: 'black',
	strokeColor: 'black',
	strokeThickness: 4,
	fill: 'yellow',
	align: 'center', 
	dropShadow: true,
    dropShadowColor: '#000000',
    dropShadowBlur: 4,
    dropShadowAngle: Math.PI / 6,
    dropShadowDistance: 6,
	wordWrap: true,
    wordWrapWidth: 800,
    lineJoin: 'round',
});

const scenarios = [
	new InterceptScenario(100),
	new InterceptScenario(80),
	new InterceptScenario(60),
	new InterceptScenario(40),
	new InterceptScenario(20),
	new TutorialScenario(zeroCutTutorial),
	new TutorialScenario(kickAndBuildTutorial),
];

// -- Event handlers

$(document).ready(function() {

	// Listen for window resize events
	$(window).resize(resize);

	// Range buttons
	$('#r25').click(setRange);
	$('#r50').click(setRange);
	$('#r100').click(setRange);
	$('#r200').click(setRange);

	// Azimuth scan buttons
	$('#az10').click(setAZScan);
	$('#az20').click(setAZScan);
	$('#az40').click(setAZScan);
	$('#az65').click(setAZScan);

	// Elevation bar buttons
	$('#bars1').click(setELBars);
	$('#bars2').click(setELBars);
	$('#bars4').click(setELBars);
	$('#bars8').click(setELBars);

	// TID Mode buttons
	$('#gnd-stab').click(setTIDMode);
	$('#ac-stab').click(setTIDMode);
	$('#attk').click(setTIDMode);

	$('#az-left').mousedown(() => {
		turnRadar(-0.6);
		if (fireInterval != null) clearInterval(fireInterval);
		fireInterval = setInterval(turn, 50, -0.6);
	});

	$('#az-left').mouseup(() => {
		clearInterval(fireInterval);
		fireInterval = null;
	});

	$('#az-right').mousedown(() => {
		turnRadar(0.6);
		if (fireInterval != null) clearInterval(fireInterval);
		fireInterval = setInterval(turn, 50, 0.6);
	});

	$('#az-right').mouseup(() => {
		clearInterval(fireInterval);
		fireInterval = null;
	});

	$('#el-up').mousedown(() => {
		elevate(0.2);
		if (fireInterval != null) clearInterval(fireInterval);
		fireInterval = setInterval(elevate, 100, 0.2);
	});

	$('#el-up').mouseup(() => {
		clearInterval(fireInterval);
		fireInterval = null;
	});

	$('#el-down').mousedown(() => {
		elevate(-0.2);
		if (fireInterval != null) clearInterval(fireInterval);
		fireInterval = setInterval(elevate, 100, -0.2);
	});

	$('#el-down').mouseup(() => {
		clearInterval(fireInterval);
		fireInterval = null;
	});

	// Add the PIXI view
	$('#display').append(app.view);

	// Enable dragging on the window bar
	$('#tomcat-ctl').draggable({ handle: '#tomcat-title' });
	$('#game-ctl').draggable({ handle: '#game-title' });
	$('#message-history').draggable({ handle: '#history-title' });

	// Enable minimizing
	$('#tomcat-min').click(() => {
		if ($('#tomcat-content').is(':visible')) {
			$('#tomcat-content').hide();
			$('#tomcat-min').removeClass('fa-caret-up');
			$('#tomcat-min').addClass('fa-caret-down');
		}
		else {
			$('#tomcat-content').show();
			$('#tomcat-min').removeClass('fa-caret-down');
			$('#tomcat-min').addClass('fa-caret-up');
		}
	});
	$('#game-min').click(() => {
		if ($('#game-content').is(':visible')) {
			$('#game-content').hide();
			$('#game-min').removeClass('fa-caret-up');
			$('#game-min').addClass('fa-caret-down');
		}
		else {
			$('#game-content').show();
			$('#game-min').removeClass('fa-caret-down');
			$('#game-min').addClass('fa-caret-up');
		}
	});
	$('#history-min').click(() => {
		if ($('#history-content').is(':visible')) {
			$('#history-content').hide();
			$('#history-min').removeClass('fa-caret-up');
			$('#history-min').addClass('fa-caret-down');
		}
		else {
			$('#history-content').show();
			$('#history-min').removeClass('fa-caret-down');
			$('#history-min').addClass('fa-caret-up');
		}
	});
	$('#splash-screen').click(() => {
		$('#splash-screen').hide();
		$('#overlay').hide();
	});

	$('#set-angels').click(() => {
		let alt = $('#angels').find(":selected").val();
		tomcat.setAltitude(ft2m(alt));
		if (tomcat.currPitch != '') $('#' + tomcat.currPitch).removeClass('pressed');
		tomcat.currPitch = '';
		messages.add('RIO: make your altitude ' + alt + ' feet.');
	});

	$('#set-speed').click(() => {
		let spd = $('#speed').find(":selected").val();
		tomcat.setSpeed(kts2ms(spd));
		messages.add('RIO: make your speed ' + spd + ' knots.');
	});

	$('#left-break').click((e) => rollTomcat(e.target.id));
	$('#left-hard').click((e) => rollTomcat(e.target.id));
	$('#left-std').click((e) => rollTomcat(e.target.id));
	$('#steady').click((e) => rollTomcat(e.target.id));
	$('#right-std').click((e) => rollTomcat(e.target.id));
	$('#right-hard').click((e) => rollTomcat(e.target.id));
	$('#right-break').click((e) => rollTomcat(e.target.id));

	$('#up-10k').click((e) => setTomcatAltitude(tomcat.pos.z + ft2m(10000), e.target.id, 'climb 10,000.'));
	$('#up-5k').click((e) => setTomcatAltitude(tomcat.pos.z + ft2m(5000), e.target.id, 'climb 5,000.'));
	$('#up-1k').click((e) => setTomcatAltitude(tomcat.pos.z + ft2m(1000), e.target.id, 'climb 1,000.'));
	$('#hold').click((e) => setTomcatAltitude(tomcat.pos.z, e.target.id, 'hold current altitude.'));
	$('#dwn-1k').click((e) => setTomcatAltitude(tomcat.pos.z - ft2m(1000), e.target.id, 'descend 1,000.'));
	$('#dwn-5k').click((e) => setTomcatAltitude(tomcat.pos.z - ft2m(5000), e.target.id, 'descend 5,000.'));
	$('#dwn-10k').click((e) => setTomcatAltitude(tomcat.pos.z - ft2m(10000), e.target.id, 'descend 10,000.'));

	// Scenario control
	for (const s of scenarios) {
		$('#scenario').append(`<option>${s.title}</option>`);
	}
	$('#select-scenario').click(() => {
		if (scenario != null) {
			scenario.cleanup();
			scenario = null;
		}
		scenario = scenarios[$('#scenario').prop('selectedIndex')];
		$('#scenario-status').html(`Current scenario: <b>${scenario.title}</b>`);
		scenario.init();
	});
	$('#clear-scenario').click(() => {
		if (scenario != null) {
			scenario.cleanup();
			$('#scenario-status').html(`Current scenario: <b>None</b>`);
			scenario = null;
		}
	});

	// Keyboard control
	let turnDelta = 0.2;
	$(document).on('keydown', (e) => {

		switch (e.key) {
		case 'ArrowLeft':
			turnRadar(-turnDelta);
			break;
		case 'ArrowRight':
			turnRadar(turnDelta);
			break;
		case 'ArrowUp':
			elevate(0.05);
			break;
		case 'ArrowDown':
			elevate(-0.05);
			break;
		case ' ':
			awg9.azimuth = 0.0;
			awg9.elevation = 0.0;
			break;
		case 'a':
			changeTomcatRoll(-1);
			break;
		case 'd':
			changeTomcatRoll(+1);
			break;
		case 's':
			rollTomcat('steady');
			break;
		}
		turnDelta += 0.05;
		if (turnDelta > 1.0) turnDelta = 1.0;
	});

	$(document).on('keyup', (e) => {
		turnDelta = 0.2;
	});

	// Animation/game loop
	let lastTime = Date.now();
	app.ticker.add((delta) => {

		// Update time
		let currTime = Date.now();
		let diffTime = (currTime - lastTime) / 1000.0; 

		// Update the fighter
		tomcat.update(diffTime);

		// Update the world
		world.update(diffTime);

		// Update the radar
		awg9.update(diffTime);

		// Update the message system
		messages.update(diffTime);

		// Update the scenario
		if (scenario) scenario.update(diffTime);

		// Save the time
		lastTime = currTime;
	});

});

// -- Main script follows
let winWidth, winHeight;

// Create the PIXI application
let app = new PIXI.Application({
	autoResize: true,
	antialias: true,
	backgroundColor: 'black',
	resolution: window.devicePixelRatio || 1
});

// Create the Tomcat
const lat = 15.0979, lon = 145.6739; // Mariana Islands
let tomcat = new Tomcat(lat, lon, 0, ft2m(25000), kts2ms(350));

// Create the AWG9 radar display
let awg9 = new AWG9(tomcat, window.innerWidth, window.innerHeight);
app.stage.addChild(awg9.display);

// Create the world
let world = new World();

// Set up the scenario
let scenario = null;

// Create the message system
let messages = new MessageSystem();
app.stage.addChild(messages.text);

// Set the correct resolution
resize();
