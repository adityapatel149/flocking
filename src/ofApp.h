#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "Particle.h"
#include "ParticleSystem.h"
#include "ofxAssimpModelLoader.h"

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);


	ofEasyCam mainCam;
	ofLight light;
	ofLight light2;

	int NUM_PARTICLES;
	ParticleSystem sys;
	Turbulence t;
	Seperation seperation;
	Alignment alignment;
	Cohesion cohesion;
	BoundaryForce boundary;

	ofxAssimpModelLoader terrain;
	ofxAssimpModelLoader boid;
	ofImage bgImageLeft, bgImageCenter, bgImageRight ;

	int currentAnimation;

	bool bHide;
	bool play;

	float h, framerate;
	glm::vec3 boxSize, boxOffset, boxMin, boxMax;

	ofxPanel gui;
	ofxGuiGroup flocking, tuning, movement;
	ofxFloatSlider turbulence, Ks, Ka, Kc;
	ofxIntSlider r1, r2, vision, max_a, min_v, max_v;
};
