#pragma once
#include "ofMain.h"
#include "ofxAssimpModelLoader.h"

struct ParticleState {
	glm::vec3 position;
	glm::vec3 velocity;
	glm::vec3 acceleration;
	glm::vec3 forces;
	float mass;
	float radius;
};

class Particle : public ofNode{
public:
	Particle();

	glm::vec3 position;
	glm::vec3 velocity;
	glm::vec3 acceleration;
	glm::vec3 forces;
	glm::mat3 rotationMatrix;

	float mass, radius;
	float Kphi, prev_angle, max_velocity, min_velocity, max_acceleration;
	ofColor color;

	void integrate(float timestep);
	void update();
	void reset();
	void setVelocity(float x, float y, float z);
	void setMinVelocity(float v);
	void setMaxVelocity(float v);
	void setPosition(float x, float y, float z);
	void setMass(float m);
	void setRadius(float r);
	void setColor(ofColor r);

	void setState(ParticleState state);
	ParticleState getState();
};