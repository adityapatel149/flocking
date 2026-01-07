
#include "omp.h"
#include "ofApp.h"
#include "ofxGui.h"

//--------------------------------------------------------------
void ofApp::setup() {
	NUM_PARTICLES = 250;
	// Bounding box for boids 
	boxSize = glm::vec3(5000, 2000, 2000);
	boxOffset = glm::vec3(0, 1600, 0);

	ofSetBackgroundColor(ofColor(131, 178, 194));
	bgImageLeft.load("pexels-soranov-18315043_left2.jpg");
	bgImageCenter.load("pexels-soranov-18315043_center2.jpg");
	bgImageRight.load("pexels-soranov-18315043_right2.jpg");

	// Setup Camera
	mainCam.setDistance(1000);
	mainCam.setNearClip(50);
	mainCam.setFarClip(50000);
	//mainCam.setFov(90);
	
	// Setup Terrain model
	terrain.load("terrain3.glb",  -1);
	float TERRAIN_SCALE = 10;
	terrain.setScale(TERRAIN_SCALE, TERRAIN_SCALE, TERRAIN_SCALE);
	terrain.setRotation(0, 180, 1, 0, -10);
	terrain.setPosition(-100, -2000, 0);
	
	//Setup Boid model
	boid.load("bird4.fbx", -1);
	boid.setPosition(0, 0, 0);
	boid.setScale(0.054, 0.054, 0.054);
	currentAnimation = 0;
	if (boid.hasAnimations()) {
		boid.setLoopStateForAllAnimations(OF_LOOP_NORMAL);
		boid.setAnimation(currentAnimation);
		boid.playAllAnimations();
	}

	
	// Setup light	
	light.setup();
	light.setPointLight();
	light.setPosition(2000, 100, 0);
	light.setDiffuseColor(ofColor(255, 200, 150));

	light2.setAreaLight(50, 50);
	light2.setPosition(-1000, 500, 00);
	light2.setAmbientColor(ofColor(255, 180, 180));


	// Setup GUI
	gui.setup();
	flocking.setName("Flocking Behaviour");
	//flocking.add(turbulence.setup("Turbulence", 0.01, 0, 0.5));
	flocking.add(Ks.setup("Seperation", 30, 0, 100));
	flocking.add(Ka.setup("Alignment", 0.02, 0, 0.1));
	flocking.add(Kc.setup("Cohesion", 0.2, 0, 1));	

	tuning.setName("Tuning Parameters");
	tuning.add(r1.setup("R1", 100, 1, 300));
	tuning.add(r2.setup("R2", 400, 301, 600));
	tuning.add(vision.setup("Vision FOV", 210, 10, 360));

	movement.setName("Movement Constraints");
	movement.add(min_v.setup("Min Velocity", 50, 0, 100));
	movement.add(max_v.setup("Max Velocity", 300, 100, 500));
	movement.add(max_a.setup("Max Acceleration", 100, 0, 1000));

	gui.add(&flocking);
	gui.add(&tuning);
	gui.add(&movement);

	// Booleans
	bHide = false;
	play = false;
	
	// Setup particle system
	std::srand(std::time(0));  // Seed the random number generator
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle p;
		// Set random position within the bounding box 
		glm::vec3 position(
			rand() % int(boxSize.x) + (boxOffset.x - boxSize.x / 2),  
			rand() % int(boxSize.y) + (boxOffset.y - boxSize.y / 2),  
			rand() % int(boxSize.z) + (boxOffset.z - boxSize.z / 2)   
		);
		p.setPosition(position.x, position.y, position.z);
		p.setVelocity(rand() % 401 - 200, rand() % 401 - 200, rand() % 401 - 200);  // Random velocity in the range [-200, 200]
		sys.add(p);
	}


	// Setup Forces
	//sys.addForce(&t);
	sys.addForce(&seperation);
	sys.addForce(&alignment);
	sys.addForce(&cohesion);

	// Boundary force to repel particle
	boxMin = boxOffset - boxSize / 2.0f;  
	boxMax = boxOffset + boxSize / 2.0f;  	
	boundary.set(500, boxMin, boxMax);
	sys.addForce(&boundary);
}

//--------------------------------------------------------------
void ofApp::update() {
	// update values from sliders
	//t.set(turbulence);
	seperation.set(Ks);
	alignment.set(Ka);
	cohesion.set(Kc);

	
	// update model animation
	boid.update();

	if (play) {

		// Parallelize the loop on multiple threads
		#pragma omp parallel for
		for (int i = 0; i < sys.particles.size(); i++) {
			framerate = ofGetFrameRate();
			h = 1.0 / framerate;
			Particle& particle = sys.particles[i];

			// Set min and max velocity from sliders
			particle.setMinVelocity(min_v);
			particle.setMaxVelocity(max_v);

			// Accumulate all forces
			float rem_a = max_a;
			glm::vec3 totalSeparation(0, 0, 0);
			glm::vec3 totalAlignment(0, 0, 0);
			glm::vec3 totalCohesion(0, 0, 0);

			#pragma omp parallel 
			{
				glm::vec3 localSeparation(0, 0, 0);
				glm::vec3 localAlignment(0, 0, 0);
				glm::vec3 localCohesion(0, 0, 0);

				#pragma omp for nowait
				for (int j = 0; j < sys.particles.size(); j++) {
					if (j != i) {
						Particle& p2 = sys.particles[j];
						glm::vec3 p1ToP2 = p2.position - particle.position;

						float cosTheta = glm::dot(glm::normalize(particle.velocity), glm::normalize(p1ToP2));
						cosTheta = glm::clamp(cosTheta, -1.0f, 1.0f);
						float angleDeg = glm::degrees(acos(cosTheta));
						
						// If p2 in FOV of particle
						if (angleDeg < vision) {
							// Distance Tuning
							float dist = glm::length(p1ToP2);
							if (dist < r2) {
								float kd = (dist > r1) ? (r2 - dist) / (r2 - r1) : 1;
								localSeparation += seperation.apply(&particle, &p2, kd);
								localAlignment += alignment.apply(&particle, &p2, kd);
								localCohesion += cohesion.apply(&particle, &p2, kd);
							}
						}												
					}
				}

				// Combine results safely, only one thread at a time, as they are shared values
				#pragma omp critical
				{
					totalSeparation += localSeparation;
					totalAlignment += localAlignment;
					totalCohesion += localCohesion;
				}
			}

			// Acceleration Priority
			glm::vec3 a(0, 0, 0);
			if (glm::length(totalSeparation) > 0.0001f) {
				a += min(rem_a, glm::length(totalSeparation)) * glm::normalize(totalSeparation);
				rem_a = max_a - glm::length(a);
			}
			if (glm::length(totalAlignment) > 0.0001f) {
				a += min(rem_a, glm::length(totalAlignment)) * glm::normalize(totalAlignment);
				rem_a = max_a - glm::length(a);
			}
			if (glm::length(totalCohesion) > 0.0001f) {
				a += min(rem_a, glm::length(totalCohesion)) * glm::normalize(totalCohesion);
			}

			particle.forces += a;
			boundary.apply(&particle);
			sys.update(&particle, h);

		}		
	}
}

//--------------------------------------------------------------
void ofApp::draw() {

	
	mainCam.begin();

	ofEnableDepthTest();


	ofPushMatrix();	
	ofRotateYDeg(101.44);
	bgImageLeft.draw(-1660, -500, -2515, 2930, 3540);

	ofRotateYDeg(-180);
	bgImageRight.draw(-1270, -500, -2605, 2930, 3540);

	ofRotateYDeg(90);
	bgImageCenter.draw(-2515, -500, -1270, 5120, 3540);
	ofPopMatrix();
	

	ofEnableLighting();
	
	//sys.draw();

	for (int i = 0; i < sys.particles.size(); i++) {
		glm::mat3 rotationMatrix = sys.particles[i].rotationMatrix;
		Particle& particle = sys.particles[i];
		glm::vec3 pos = particle.position;
			//boid.setPosition(pos.x, pos.y, pos.z);
		ofPushMatrix();  
		ofTranslate(pos.x, pos.y, pos.z);
		ofMultMatrix(glm::mat4(rotationMatrix));
		boid.drawFaces();
		ofPopMatrix();  
	}

	light.enable();
	light2.enable();

	terrain.drawFaces();

	ofSetColor(255); 
	ofDrawBitmapString("FPS: " + ofToString(ofGetFrameRate()), 10, 20);

	light.disable();
	ofDisableLighting();
	ofDisableDepthTest();
	mainCam.end();

	if (!bHide) gui.draw();

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	switch (key) {

		// 32 is int code for spacebar
	case (32): {
		play = !play;
		break;
	};
	case ('h'): {
		bHide = !bHide;
		break;
	};
	case ('r'): {
		sys.reset();
		// Setup particle system
		std::srand(std::time(0));  // Seed the random number generator
		for (int i = 0; i < NUM_PARTICLES; i++) {
			Particle& p = sys.particles[i];
			// Set random position within the bounding box 
			glm::vec3 position(
				rand() % int(boxSize.x) + (boxOffset.x - boxSize.x / 2),
				rand() % int(boxSize.y) + (boxOffset.y - boxSize.y / 2),
				rand() % int(boxSize.z) + (boxOffset.z - boxSize.z / 2)
			);
			p.setPosition(position.x, position.y, position.z);
			p.setVelocity(rand() % 401 - 200, rand() % 401 - 200, rand() % 401 - 200); 
		}
		break;
	};
	case (OF_KEY_RETURN): {
		break;
	};
	default: break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
