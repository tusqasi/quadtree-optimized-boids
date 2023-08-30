/* TUSqasi HARisora
 https://github.com/tusqasi
 Forged from Daniel Shiffman with ♥️
 http://codingtra.in

 QuadTree from Daniel Shiffman
 https://www.youtube.com/watch?v=z0YFFg_nBjw

 Boids
 https://www.youtube.com/watch?v=mhjuuHl6qHM

 For more:
 https://github.com/CodingTrain/QuadTree
*/
'use strict';
class Boid {
	// Setup a boid
	constructor() {
		this.position = createVector(random(100, W - 100), random(100, H - 100));
		this.velocity = p5.Vector.random2D();
		this.velocity.setMag(random(2, 4));
		this.acceleration = createVector();

		this.alignment_sensitivity = 1;
		this.cohesion_sensitivity = 0.9;
		this.separation_sensitivity = 0.8;

		this.maxForce = 0.3;
		this.boid_color = 'white';
	}

	render() {
		strokeWeight(1)
		push();
		fill(this.boid_color);
		stroke(this.boid_color);
		push();
		translate(this.position);
		rotate(this.velocity.heading());
		noFill();
		line(SIZE * 10, 0, 0, SIZE * 10);
		line(SIZE * 10, 0, 0, SIZE * -10);

		line(SIZE * 8, 0, 0, SIZE * 10);
		line(SIZE * 8, 0, 0, SIZE * -10);
		// stroke("#ffffff09");
		// circle(0,0,PerceptionRadius);

		pop();
		pop();

	}

	update() {
		this.position.add(this.velocity);
		this.velocity.add(this.acceleration);
		// this.velocity.limit(this.maxSpeed);
		this.acceleration.mult(0);
	}

	flock(boids) {
		let alignment = this.alignment(boids);
		let cohesion = this.cohesion(boids);
		let seperation = this.seperation(boids);

		// alignment.mult(alignmentSlider.value());
		// cohesion.mult(cohesionSlider.value());
		// seperation.mult(seperationSlider.value());

		// alignment.mult(0.1);
		// cohesion.mult(0.1);
		// seperation.mult(0.1);
		this.acceleration.add(alignment);
		this.acceleration.add(cohesion);
		this.acceleration.add(seperation);
	}
	/**
	* Point to where neighbourhood is pointing
	* @param {Boid[]} boids
	* @return {number} 
	*/
	alignment(boids) {
		// Alignment
		// go where the neighbours are going

		let avg_heading = createVector();

		for (let boid of boids) {
			avg_heading.add(boid.velocity);
		}
		if (boids.length > 0) {
			avg_heading.div(boids.length);
			avg_heading.setMag(this.alignment_sensitivity);
			avg_heading.sub(this.velocity);
			avg_heading.limit(this.maxForce);
		}
		return avg_heading;
	}
	/**
	* Stay near the center of local neighbourhood
	* @param {array} boids
	* @return {number} 
	*/
	cohesion(boids) {
		// Cohesion
		// go toward the center of the flock

		let avg_position = createVector();
		for (let boid of boids) {
			// adding positions together
			avg_position.add(boid.position);
		}
		if (boids.length > 0) {
			avg_position.div(boids.length); // Average
			avg_position.sub(this.position); // Pointing towards center of the flock
			avg_position.setMag(this.cohesion_sensitivity); // Normalize
			avg_position.sub(this.velocity); //
			avg_position.limit(this.maxForce);
		}
		return avg_position;
	}
	/**
	* Point away from neighbouring boids
	* @param {array} boids
	* @return {number} 
	*/
	seperation(boids) {
		// How far can boid "see"

		let avg_diff = createVector();
		for (let boid of boids) {
			let distance = this.position.dist(boid.position);
			let vector_to_this = p5.Vector.sub(this.position, boid.position);
			vector_to_this.div(distance * distance); // Get the unit vector, i.e direction of other boid towards this boid.
			avg_diff.add(vector_to_this);
		}
		if (boids.length > 0) {
			avg_diff.div(boids.length); // Average
			avg_diff.setMag(this.separation_sensitivity); // Normalize
			avg_diff.sub(this.velocity); //
			avg_diff.limit(this.maxForce);
		}
		return avg_diff;
	}

	wrap() {
		// x > screen. wrap
		// x < screen. wrap
		if (this.position.x >= W) {
			this.position.x = 0;
		} else if (this.position.x <= 0) {
			this.position.x = W;
		}

		// y > screen. wrap
		// y < screen. wrap
		if (this.position.y >= H) {
			this.position.y = 0;
		} else if (this.position.y <= 0) {
			this.position.y = H;
		}
	}
	focus() {
		push();
		strokeWeight(1);
		noFill();
		circle(this.position.x, this.position.y, 50);
		// this is color blue.
		stroke(BLUE);
		this.draw();
		pop();
	}


	/**
	* Optimaly flocks the boids
	* @param {array} boids
	*/
	optimal_flock(boids) {
		// let PerceptionRadius = PerceptionRadius;
		let distance = 0;
		let avg_position = createVector();
		let avg_heading = createVector();
		let avg_diff = createVector();
		for (let other_boid of boids) {
			if (other_boid == this) {
				continue;
			}
			distance = this.position.dist(other_boid.position);
			if (distance < PerceptionRadius) {
				// Adding positions together
				avg_position.add(other_boid.position);
				avg_heading.add(other_boid.velocity);
				let vector_to_this = p5.Vector.sub(this.position, other_boid.position);
				vector_to_this.div(distance * distance); // Get the unit vector, i.e direction of other boid towards this boid.
				avg_diff.add(vector_to_this / Math.sqrt(distance));
				avg_heading.add(other_boid.velocity);
			}

		}
		if (boids.length > 0) {
			avg_position.div(boids.length); // Average
			avg_position.sub(this.position); // Pointing towards center of the flock
			avg_position.setMag(this.cohesion_sensitivity); // Normalize
			avg_position.sub(this.velocity); //
			// avg_position.limit(this.maxForce);
			avg_heading.div(boids.length);
			avg_heading.setMag(this.alignment_sensitivity);
			avg_heading.sub(this.velocity);
			// avg_heading.limit(this.maxForce);
			avg_diff.div(boids.length); // Average
			avg_diff.setMag(this.separation_sensitivity); // Normalize
			avg_diff.sub(this.velocity); //
			// avg_diff.limit(this.maxForce);
		}
		const factor = 1;
		avg_diff.mult(factor);
		avg_heading.mult(factor);
		avg_position.mult(factor);

		this.acceleration.add(avg_diff);
		this.acceleration.add(avg_heading);
		this.acceleration.add(avg_position);
		this.acceleration.limit(this.maxForce)
	}
}
class Point {
	/**
	* @param {number} x 
	* @param {number} y 
	* @param {Boid} userData
	*/
	constructor(x, y, userData) {
		this.x = x;
		this.y = y;
		this.userData = userData;
	}
}

class Rectangle {
	/**
	* A rectangle region 
	* @constructor
	* @param {number} x - x position
	* @param {number} y - y position
	* @param {number} w - width
	* @param {number} h - height
	* */
	constructor(x, y, w, h) {
		this.x = x;
		this.y = y;
		this.w = w;
		this.h = h;
	}

	/**
	* @param {Point} point
	*/
	contains(point) {
		return (point.x >= this.x - this.w &&
			point.x <= this.x + this.w &&
			point.y >= this.y - this.h &&
			point.y <= this.y + this.h);
	}


	/**
	* @param  {Circle} range
	*/
	intersects(range) {
		return !(range.x - range.w > this.x + this.w ||
			range.x + range.w < this.x - this.w ||
			range.y - range.h > this.y + this.h ||
			range.y + range.h < this.y - this.h);
	}


}

// circle class for a circle shaped query
class Circle {
	/**
	* A circular region 
	* @constructor
	* @param {number} x - x position
	* @param {number} y - y position
	* @param {number} r - radius
	* */
	constructor(x, y, r) {
		this.x = x;
		this.y = y;
		this.r = r;
		this.rSquared = this.r * this.r;
	}

	/**
	* Checks if the point lies in this region
	* @param {Point} point
	*/
	contains(point) {
		// check if the point is in the circle by checking if the euclidean distance of
		// the point and the center of the circle if smaller or equal to the radius of
		// the circle
		let d = Math.pow((point.x - this.x), 2) + Math.pow((point.y - this.y), 2);
		return d <= this.rSquared;
	}

	/**
	* Check if given circle intersects this circle
	* @param  {Circle} range
	*/
	intersects(range) {

		var xDist = Math.abs(range.x - this.x);
		var yDist = Math.abs(range.y - this.y);

		// radius of the circle
		var r = this.r;

		var w = range.w;
		var h = range.h;

		var edges = Math.pow((xDist - w), 2) + Math.pow((yDist - h), 2);

		// no intersection
		if (xDist > (r + w) || yDist > (r + h))
			return false;

		// intersection within the circle
		if (xDist <= w || yDist <= h)
			return true;

		// intersection on the edge of the circle
		return edges <= this.rSquared;
	}
}

class QuadTree {
	/**
	* @param {Rectangle} boundary
	* @param {number} capacity
	*/
	constructor(boundary, capacity) {
		if (!boundary) {
			throw TypeError('boundary is null or undefined');
		}
		if (!(boundary instanceof Rectangle)) {
			throw TypeError('boundary should be a Rectangle');
		}
		if (typeof capacity !== 'number') {
			throw TypeError(`capacity should be a number but is a ${typeof capacity}`);
		}
		if (capacity < 1) {
			throw RangeError('capacity must be greater than 0');
		}
		this.boundary = boundary;
		this.capacity = capacity;
		this.points = [];
		this.divided = false;
		this.northeast = undefined;
		this.southeast = undefined;
		this.southwest = undefined;
		this.northwest = undefined;
	}

	subdivide() {
		let x = this.boundary.x;
		let y = this.boundary.y;
		let w = this.boundary.w / 2;
		let h = this.boundary.h / 2;

		let ne = new Rectangle(x + w, y - h, w, h);
		this.northeast = new QuadTree(ne, this.capacity);
		let nw = new Rectangle(x - w, y - h, w, h);
		this.northwest = new QuadTree(nw, this.capacity);
		let se = new Rectangle(x + w, y + h, w, h);
		this.southeast = new QuadTree(se, this.capacity);
		let sw = new Rectangle(x - w, y + h, w, h);
		this.southwest = new QuadTree(sw, this.capacity);

		this.divided = true;
	}

	/**
	* Inserts a point into the tree
	* @param {Point} point 
	* @returns {boolean}
	*/
	insert(point) {
		if (!this.boundary.contains(point)) {
			return false;
		}

		if (this.points.length < this.capacity) {
			this.points.push(point);
			return true;
		}

		if (!this.divided) {
			this.subdivide();
		}

		if (this.northeast.insert(point) || this.northwest.insert(point) ||
			this.southeast.insert(point) || this.southwest.insert(point)) {
			return true;
		}
	}
	/**
	* Finds the points in the range 
	* @param {Rectangle } range
	* @param {Point[]} found
	* @returns {Point[]} found
	*/
	query(range, found) {
		if (!found) {
			found = [];
		}

		if (!range.intersects(this.boundary)) {
			return found;
		}

		for (let p of this.points) {
			if (range.contains(p)) {
				found.push(p);
			}
		}
		if (this.divided) {
			this.northwest.query(range, found);
			this.northeast.query(range, found);
			this.southwest.query(range, found);
			this.southeast.query(range, found);
		}

		return found;
	}

	show() {
		stroke(255);
		noFill();
		rectMode(CENTER);
		strokeWeight(1)
		rect(this.boundary.x, this.boundary.y, this.boundary.w * 2, this.boundary.h * 2);
		for (let p of this.points) {
			strokeWeight(3);
			stroke(200);
			point(p.x, p.y);
		}

		if (this.divided) {
			this.northeast.show();
			this.northwest.show();
			this.southeast.show();
			this.southwest.show();
		}
	}

}

const SIZE = .5;

// Height and Width of the canvas
const W = 1920 * .88;
const H = 1080 * .88;

/**
* How far can a boid see
* @type {number} PerceptionRadius 
*/
const PerceptionRadius = 400;

/**
* Stores all the boids
* @type {Boid[]} boids
*/
let boids = [];

/**
* Number of boids
* @type {number} N_BOIDS
*/
let N_BOIDS = 400;
let img;
function setup() {
	createCanvas(W, H);
	for (let i = 0; i < N_BOIDS; i++) {
		boids[i] = new Boid();
	}
	// boids[3].boid_color = 'red'
}
let qtree;
function draw() {
	background(10);
	let boundary = new Rectangle(W / 2, H / 2, W, H);
	qtree = new QuadTree(boundary, 3);
	for (let boid of boids) {
		let pt = new Point(boid.position.x, boid.position.y, boid);
		qtree.insert(pt);

		boid.update();
		boid.wrap();
		boid.render();
	}
	for (let p of boids) {
		let range = new Rectangle(p.position.x, p.position.y, PerceptionRadius, PerceptionRadius);
		let boidsVisible = qtree.query(range);
		boidsVisible = boidsVisible.filter((i) => i.userData !== p)
		boidsVisible = boidsVisible.map((p) => p.userData)
		// if(p == boids[3]){
		// 	for(let x of boidsVisible){
		// 	x.boid_color = 'green'
		// 	}
		// }
		p.optimal_flock(boidsVisible);
	}
	// qtree.show();
}
