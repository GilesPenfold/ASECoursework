#include "Boid.h"
#include <random>
#include <math.h>
#include <ngl/ShaderLib.h>
#include <ngl/VAOPrimitives.h>




Boid::Boid(){}

Boid::Boid(ngl::Vec3 _pos, float _ms, float _mf)
{
    m_pos = _pos;
    m_vel = ngl::Vec3(0,0,0);
    // Generate random acceleration between -1 and 1
    m_acc = ngl::Vec3(static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2.0f))-1.0f, static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2.0f))-1.0f, 0);
    m_colour = ngl::Vec3(1,1,1);

    m_r = 58.0f;
    m_maxSpeed = _ms;
    m_maxForce = _mf;

}
Boid::Boid(ngl::Vec3 _pos, ngl::Vec3 _vel, ngl::Vec3 _acc, float _r, float _mf, float _ms)
{
    m_pos = _pos;
    m_vel = _vel;
    m_acc = _acc;
    m_colour = ngl::Vec3(1,1,1);
    m_r = _r;
    m_maxSpeed = _ms;
    m_maxForce = _mf;
}

void Boid::Run(std::vector<std::shared_ptr<Boid>>* _boids)
{
    Flock(_boids);

    Update();

    Borders();

}



void Boid::Draw(ngl::Camera *_cam)
{

    ngl::ShaderLib *shader=ngl::ShaderLib::instance();

    ngl::Mat4 MV;
    ngl::Mat4 MVP;
    ngl::Mat3 normalMatrix;
    ngl::Mat4 M;
    ngl::Transformation t;
    t.setPosition(m_pos.m_x,m_pos.m_y,m_pos.m_z);


    M=  t.getMatrix();
    MV=  _cam->getViewMatrix()*M;
    MVP= _cam->getVPMatrix()*M;
    normalMatrix=MV;
    normalMatrix.inverse().transpose();
    shader->setUniform("MVP",MVP);
    shader->setUniform("normalMatrix",normalMatrix);
    // get the VBO instance and draw the built in teapot
    ngl::VAOPrimitives *prim=ngl::VAOPrimitives::instance();
    prim->draw("sphere");


}

void Boid::Update()
{

    // Accelerate

    m_vel += m_acc;

    // Adjust speed to limit

    m_vel = Limit(m_vel, m_maxSpeed);

    if(Magnitude(m_vel) == 0.0f)
        m_vel = ngl::Vec3(m_maxSpeed,m_maxSpeed,0.0f);

    m_pos += m_vel;


    m_acc.null();

}

void Boid::Flock(std::vector<std::shared_ptr<Boid>>* _boids)
{
    ngl::Vec3 S = Separate(_boids);
    ngl::Vec3 A = Align(_boids);
    ngl::Vec3 C = Cohesion(_boids);

    // Weight the forces (the network will be adjusting these values)

    S = S * 2.0f;
    A = A * 1.5f;
    C = C * 1.0f;

    m_colour = (m_colour + (ngl::Vec3((A.m_x+A.m_y),(A.m_x + A.m_y), (A.m_x + A.m_y))/m_colour)/30.0f);

    if(m_colour.m_x < 0.5f)
        m_colour.m_x = 0.5f;

    if(m_colour.m_y < 0.5f)
        m_colour.m_y = 0.5f;

    if(m_colour.m_z < 0.5f)
        m_colour.m_z = 0.5f;

    if(m_colour.m_x > 1.0f)
        m_colour.m_x = 1.0f;

    if(m_colour.m_y > 1.0f)
        m_colour.m_y = 1.0f;

    if(m_colour.m_z > 1.0f)
        m_colour.m_z = 1.0f;

    // Apply these to the acceleration

    m_acc += S;
    m_acc += A;
    m_acc += C;
}

void Boid::Seek(ngl::Vec3 _target)
{
    m_acc += Steer(_target, false);
}

void Boid::Arrive(ngl::Vec3 _target)
{
    m_acc += Steer(_target, true);
}

ngl::Vec3 Boid::Steer(ngl::Vec3 _target, bool _slow)
{
    ngl::Vec3 SVec;
    ngl::Vec3 loc = _target - m_pos;


    float dist = Magnitude(loc);

    if(dist>0)
    {
        loc /= dist;

        // Some dampening

        if(_slow && dist < 100.0f)
        {
            loc *= (m_maxSpeed * (dist/100.0f));
        }
        else
        {
            loc *= m_maxSpeed;
        }

        SVec = loc - m_vel;

        SVec = Limit(SVec, m_maxForce);
    }
    else
    {
        SVec = ngl::Vec3(0,0,0);
    }

    return SVec;
}

void Boid::Borders()
{

    float adj = 1.5f;
    if(m_pos.m_x < -m_r-2)
        m_pos.m_x = m_r;
    if(m_pos.m_y > m_r/adj +1)
        m_pos.m_y = -m_r/adj;
    if(m_pos.m_x > m_r +2)
        m_pos.m_x = -m_r;
    if(m_pos.m_y < -m_r/adj -1)
        m_pos.m_y = m_r/adj;
}

// Rules of Flocking

ngl::Vec3 Boid::Separate(std::vector<std::shared_ptr<Boid>>* _boids)
{
    float SeparationLimit = 25.0f;

    ngl::Vec3 _return = ngl::Vec3(0,0,0);

    int count = 0;

    for(auto &b : *_boids)
    {
        float dist = Magnitude(m_pos - b->m_pos);

        if(dist > 0 && dist < SeparationLimit)
        {
            ngl::Vec3 diff = m_pos - b->m_pos;

            diff = Normalize(diff);

            diff /= dist;

            _return += diff;

            count++;
        }
    }

    if(count > 0)
    {
        _return /= (float)count;
    }

    return _return;
}

ngl::Vec3 Boid::Align(std::vector<std::shared_ptr<Boid>>* _boids)
{
    ngl::Vec3 _return = ngl::Vec3(0,0,0);

    float NeighbourDistance = 25.0f;

    int count = 0;

    for (auto &b : *_boids)
    {
        float dist = Magnitude(m_pos - b->m_pos);

        if(dist > 0 && dist < NeighbourDistance)
        {
            _return += b->m_vel;
            count++;
        }
    }


    if(count>0)
    {
        _return /= (float)count;
        _return = Limit(_return, m_maxForce);
    }

    return _return;
}
ngl::Vec3 Boid::Cohesion(std::vector<std::shared_ptr<Boid>>* _boids)
{
    float NeighbourDistance = 25.0f;

    ngl::Vec3 _return = ngl::Vec3(0,0,0);

    int count = 0;

    for (auto &b : *_boids)
    {
        float dist = Magnitude(m_pos - b->m_pos);

        if(dist > 0 && dist < NeighbourDistance)
        {
            _return += b->m_pos;
            count++;
        }
    }


    if(count > 0)
    {
        _return /= (float)count;

        return Steer(_return, false);
    }

    return _return;
}





// Useful Functions


float Boid::Magnitude(ngl::Vec3 _input){return (float)sqrt(_input.m_x*_input.m_x + _input.m_y*_input.m_y + _input.m_z*_input.m_z);};
ngl::Vec3 Boid::Normalize(ngl::Vec3 _input)
{
    ngl::Vec3 _return = _input;
    float _mag = Magnitude(_input);

    if(_mag > 0)
    {
        _return /= _mag;
    }

    return _return;
}

ngl::Vec3 Boid::Limit(ngl::Vec3 _input, float _val)
{
    ngl::Vec3 _return;
    float boid_mag = Magnitude(_input);

    if(boid_mag > _val)
    {
        _return = Normalize(_input);

        _return *= m_maxSpeed;
    }

    return _return;
}
