#include "Flock.h"


#include <random>

Flock::Flock()
{

}


void Flock::Run()
{
    for (auto &b : m_boids)
    {
        b->Run(&m_boids);
    }
}


void Flock::Add(int _i)
{
    for (int i = 0; i < _i; i++)
    {
        m_boids.push_back(std::make_unique<Boid>(ngl::Vec3(static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0f))-5.0f, static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0f))-5.0f, 0),0.3f,0.05f));
        //std::cout << m_boids.back()->m_pos.m_x << ", " << m_boids.back()->m_pos.m_y << ", " << m_boids.back()->m_pos.m_z <<'\n';

    }
}

void Flock::Draw( ngl::Camera *_cam, ngl::ShaderLib *shader )
{
    for (auto &b : m_boids)
    {
        shader->setUniform("Colour",b->m_colour.m_x,b->m_colour.m_y,b->m_colour.m_z,1.0f);
        b->Draw( _cam);
    }
}
