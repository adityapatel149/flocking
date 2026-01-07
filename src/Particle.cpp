#include "Particle.h"
#include "ofxAssimpModelLoader.h"

Particle::Particle() {
	position = glm::vec3(0, 0, 0);
	velocity = glm::vec3(0, 1, 0);
	acceleration = glm::vec3(0, 0, 0);
	forces = glm::vec3(0, 0, 0);
	mass = 1;
	radius = 10;
	color = ofColor::black;
	min_velocity = 0;
	max_velocity = 10;
	max_acceleration = 10;
	Kphi = -0.2;
	prev_angle = 0;
	rotationMatrix = glm::mat3(1.0f);	
}


void Particle::update() {

	// Get local axis and Handle NaN values
	glm::vec3 ux = glm::normalize(velocity);
	if (isnan(glm::length(ux))) ux = glm::vec3(1, 0, 0);
	glm::vec3 uy = glm::normalize(glm::cross(velocity, acceleration));
	if (isnan(glm::length(uy))) uy = glm::vec3(0, 1, 0);
	glm::vec3 uz = glm::cross(ux, uy);

	glm::vec3 av = glm::dot(acceleration, ux) * ux;
	glm::vec3 at = acceleration - av;

	// Steering angle, based on tangential acceleration and uz
	float angle = Kphi * glm::atan(glm::dot(at, uz));
	angle = (angle + prev_angle) * 0.5;
	prev_angle = angle;

	// (w,x,y,z) - w is scalar part, x,y,z are vector components of quaternion
	glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // no rotation (identity matrix)

	// First, align the particle to the velocity direction
	if (glm::length(velocity) > 0.0001f) {
		glm::vec3 forward = glm::normalize(velocity);
		glm::vec3 right = glm::normalize(glm::cross(glm::vec3(0, 1, 0), forward));
		glm::vec3 up = glm::cross(forward, right);

		rotationMatrix = glm::mat3(right, up, forward);
	}

	// Apply rotation based on acceleration 
	if (glm::length(acceleration) > 0.0001f) {
		if (glm::length(velocity) > 0.0001f && glm::length(acceleration) > 0.0001f) {
			glm::vec3 rotationAxis = glm::cross(velocity, acceleration);
			// Calculate Rotation if rotation axis is non-zero
			if (glm::length(rotationAxis) > 0.0001f) {
				rotationAxis = glm::normalize(rotationAxis);
				// Apply rotation based on acceleration direction and steering angle
				rotation = glm::rotate(rotation, angle, rotationAxis);
			}
		}
		// Apply accelration-based rotation
		rotationMatrix *= glm::mat3_cast(rotation);
	}

}

void Particle::setVelocity(float x, float y, float z) {
	velocity = glm::vec3(x, y, z);
}
void Particle::setPosition(float x, float y, float z) {
	position = glm::vec3(x, y, z);
}

void Particle::setMinVelocity(float v) {
	min_velocity = v;
}
void Particle::setMaxVelocity(float v) {
	max_velocity = v;
}
void Particle::setMass(float m) {
	mass = m;
}
void Particle::setColor(ofColor c) {
	color = c;
}

void Particle::setRadius(float r) {
	radius = r;
}

void Particle::reset() {
	position = glm::vec3(0, 0, 0);
	velocity = glm::vec3(0, 1, 0);
	acceleration = glm::vec3(0, 0, 0);
	forces = glm::vec3(0, 0, 0);
	mass = 1;
	radius = 10;
	color = ofColor::black;
	min_velocity = 0;
	max_velocity = 10;
	max_acceleration = 10;
	Kphi = -0.25;
	prev_angle = 0;
	rotationMatrix = glm::mat3(1.0f);
}

ParticleState Particle::getState() {
	return { position, velocity, acceleration, forces, mass, radius };
}

void Particle::setState(ParticleState state) {
	this->position = state.position;
	this->velocity = state.velocity;
	this->acceleration = state.acceleration;
	this->forces = state.forces;
	this->mass = state.mass;
	this->radius = state.radius;
}

void Particle::integrate(float timestep) {

	// timestep h
	float h = timestep;

	// using Improved Euler
	acceleration = forces * (1.0 / mass);

	glm::vec3 predicted_velocity = (velocity + (acceleration * h));
	position += (0.5 * (velocity + predicted_velocity) * h);

	// Current magnitude of velocity
	float v = glm::length(predicted_velocity);
	
	// Clamp v to be between min and max velocity
	v = glm::clamp(v, min_velocity, max_velocity);	
	velocity = v * glm::normalize(predicted_velocity);
	
	// clear forces on particle. They get re-added each step
	forces = glm::vec3(0, 0, 0);
}