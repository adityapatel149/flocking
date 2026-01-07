#pragma once
#include "ofMain.h"
#include "Particle.h"

class ParticleForce {
public:
    bool applyOnce = false;
    bool applied = false;
    virtual void updateForce(Particle*) = 0;
};

class ParticleSystem {
public:
    void add(const Particle&);
    void addForce(ParticleForce*);
    void remove(int);
    void update(Particle*, float);
    void reset();
    void draw();

    std::vector<Particle> particles;
    std::vector<ParticleForce*> forces;
};

// Force classes
class Turbulence : public ParticleForce {
    float t;
public:
        Turbulence(const float t);
        Turbulence();
        void set(const float t);
        void updateForce(Particle* partcle) override;
};


class Seperation : public ParticleForce {
    float k;
    glm::vec3 force;
public:
    Seperation();
    Seperation(const float k);
    void set(const float k);
    glm::vec3 apply(Particle* p1, Particle* p2, float kd);
    void updateForce(Particle* p1) override;
};


class Alignment : public ParticleForce {
    float k;
    glm::vec3 force;
public:
    Alignment();
    Alignment(const float k);
    void set(const float k);
    glm::vec3 apply(Particle* p1, Particle* p2, float kd);
    void updateForce(Particle* p1) override;
};


class Cohesion : public ParticleForce {
    float k;
    glm::vec3 force;
public:
    Cohesion();
    Cohesion(const float k);
    void set(const float k);
    glm::vec3 apply(Particle* p1, Particle* p2, float kd);
    void updateForce(Particle* p1) override;
};


class BoundaryForce : public ParticleForce {
    float magnitude;
    glm::vec3 force;
    glm::vec3 boxMin;
    glm::vec3 boxMax;
public:
    BoundaryForce(const float k, const glm::vec3 boxMin, const glm::vec3 boxMax);
    BoundaryForce();
    void set(const float k, const glm::vec3 boxMin, const glm::vec3 boxMax);
    glm::vec3 apply(Particle* p);
    void updateForce(Particle* partcle) override;
};




