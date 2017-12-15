#ifndef FLOCK_H
#define FLOCK_H

#include <vector>
#include <Boid.h>
#include <ngl/ShaderLib.h>

class Flock
{
public:
    explicit Flock();
    ~Flock() = default;

    Flock(const Flock&) = delete;
    Flock& operator=(const Flock&) = delete;

    void Draw( ngl::Camera *_cam, ngl::ShaderLib *shader  );
    void Run();
    void Add(int _i);


    std::vector<std::shared_ptr<Boid>> m_boids;

};

#endif // FLOCK_H
