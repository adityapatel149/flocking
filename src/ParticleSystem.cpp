#include "ParticleSystem.h"


// Particle System constructor and methods
void ParticleSystem::add(const Particle& p) {
    particles.push_back(p);
}

void ParticleSystem::addForce(ParticleForce* f) {
    forces.push_back(f);
}

void ParticleSystem::remove(int i) {
    particles.erase(particles.begin() + i);
}

void ParticleSystem::reset() {

    // Reset system forces
    for (int i = 0; i < forces.size(); i++) {
        if (forces[i]->applyOnce) {
            forces[i]->applied = true;
        }
        else {
            forces[i]->applied = false;
        }
    }

    // Reset particles
    for (int i = 0; i < particles.size(); i++) {
        particles[i].reset();
    }
}

void ParticleSystem::update(Particle* particle, float timestep) {

    // update all forces on particle    
    for (int i = 0; i < forces.size(); i++) {
        if (!forces[i]->applied) {
            forces[i]->updateForce(particle);

            // update forces only applied once to "applied"
            if (forces[i]->applyOnce) {
                forces[i]->applied = true;
            }
        }
    }

    // integrate particle
    particle->integrate(timestep);
    particle->update();
}

void ParticleSystem::draw() {
    /*for (int i = 0; i < particles.size(); i++) {
        particles[i].draw();
    }*/
}



// Turbulence constructor and methods
Turbulence::Turbulence(const float t) { this->t = t; }
Turbulence::Turbulence() { this->t = 1; }
void Turbulence::set(const float t) { this->t = t; }
void Turbulence::updateForce(Particle* particle) {    
    particle->forces += glm::vec3(
        (rand() % (2 * int(t) + 1)) - t,
        (rand() % (2 * int(t) + 1)) - t,
        (rand() % (2 * int(t) + 1)) - t 
    );
}


// Seperation
Seperation::Seperation(const float k) : k(k) {
    applyOnce = true;
    applied = true;
    force = glm::vec3(0,0,0);
};
Seperation::Seperation() : k(0.5) {
    applyOnce = true;
    applied = true;
};
void Seperation::set(const float k) { this->k = k; }
glm::vec3 Seperation::apply(Particle* p1, Particle* p2, float kd){
   applied = false;
   glm::vec3 x = p2->position - p1->position;
   float d = max(0.1f, glm::length(x));   
   force = -kd * (k / d) * glm::normalize(x) * 40; //multiplying 40 just to scale the value and make forces stronger
   return force;
}
void Seperation::updateForce(Particle* particle) {
   //particle->forces += force;
}


// Alignment or Velocity Matching
Alignment::Alignment(const float k) : k(k) {
    applyOnce = true;
    applied = true;
    force = glm::vec3(0,0,0);
};
Alignment::Alignment() : k(0.5) {
    applyOnce = true;
    applied = true;
};
void Alignment::set(const float k) { this->k = k; }
glm::vec3 Alignment::apply(Particle* p1, Particle* p2, float kd){
   applied = false;      
   force = kd * k * (p2->velocity - p1->velocity);
   return force;
}
void Alignment::updateForce(Particle* particle) {
   // particle->forces += force;
}


// Cohesion or Centering
Cohesion::Cohesion(const float k) : k(k) {
    applyOnce = true;
    applied = true;
    force = glm::vec3(0,0,0);
};
Cohesion::Cohesion() : k(0.5) {
    applyOnce = true;
    applied = true;
};
void Cohesion::set(const float k) { this->k = k; }
glm::vec3 Cohesion::apply(Particle* p1, Particle* p2, float kd){
   applied = false;      
   force = kd * k * (p2->position - p1->position);
   return force;
}
void Cohesion::updateForce(Particle* particle) {
    //particle->forces += force;
}


// Boundary Force

BoundaryForce::BoundaryForce(const float k, const glm::vec3 boxMin, const glm::vec3 boxMax) {
    applyOnce = true;
    applied = true;
    this->boxMin = boxMin;
    this->boxMax = boxMax;
    this->magnitude = k;
    this->force = glm::vec3(0, 0, 0);
}

BoundaryForce::BoundaryForce() {
    applyOnce = true;
    applied = true;
    this->boxMin = glm::vec3(0,0,0);
    this->boxMax = glm::vec3(100, 100, 100);
    this->magnitude = 1;
    this->force = glm::vec3(0, 0, 0);
}

void BoundaryForce::set(const float k, const glm::vec3 boxMin, const glm::vec3 boxMax) {
    applyOnce = true;
    applied = true;
    this->boxMin = boxMin;
    this->boxMax = boxMax;
    this->magnitude = k;
}

glm::vec3 BoundaryForce::apply(Particle* p) {
    applied = false;
    force = glm::vec3(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 3; ++i) {
        float& pos = p->position[i];
        float min = boxMin[i];
        float max = boxMax[i];

        // Apply repelling force if the position is outside the box
        if (pos < min) {
            force[i] = min - pos;
        }
        else if (pos > max) {
            force[i] = max - pos;
        }
    }
    return force;
}

void BoundaryForce::updateForce(Particle* particle) {    
    particle->forces += force * magnitude;
}


