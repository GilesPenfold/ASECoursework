#ifndef BOID_H
#define BOID_H
#include <ngl/Vec3.h>
#include <vector>
#include <ngl/Transformation.h>
#include <ngl/Camera.h>

/*
 * Based off of the code from http://processingjs.org/learning/topic/flocking/
 */

class Boid
{
public:
    Boid();
    Boid(ngl::Vec3 _pos, float _ms, float _mf);
    Boid(ngl::Vec3 _pos, ngl::Vec3 _vel, ngl::Vec3 _acc, float _r, float _mf, float _ms);

    float Magnitude(ngl::Vec3 _input);
    ngl::Vec3 Normalize(ngl::Vec3 _input);
    ngl::Vec3 Limit(ngl::Vec3 _input, float _val);

    void Run(std::vector<std::shared_ptr<Boid>>* _boids);

    void Update();
    void Draw(  ngl::Camera *_cam );
    void Flock(std::vector<std::shared_ptr<Boid>>* _boids);

    void Seek(ngl::Vec3 _target);
    void Arrive(ngl::Vec3 _target);

    ngl::Vec3 Steer(ngl::Vec3 _target, bool _slow);
    void Borders();

    // Rules of Flocking

    ngl::Vec3 Separate(std::vector<std::shared_ptr<Boid>>* _boids);
    ngl::Vec3 Align(std::vector<std::shared_ptr<Boid>>* _boids);
    ngl::Vec3 Cohesion(std::vector<std::shared_ptr<Boid>>* _boids);

    ngl::Vec3 OptimizedRules(std::vector<std::shared_ptr<Boid>> _boids);


    ngl::Vec3 m_pos; // Position
    ngl::Vec3 m_vel; // Velocity
    ngl::Vec3 m_acc; // Acceleration

    ngl::Vec3 m_colour; // Colour values

    float m_r; // Radius of screen
    float m_maxForce; // Maximum allowed steering force
    float m_maxSpeed; // Maximum allowed speed

};

#endif // BOID_H
