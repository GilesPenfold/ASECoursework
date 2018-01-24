
# Genetic Flocking - Giles Penfold, MSc CAVE ASE & CGI Tech Assignment 2017/18

#### This is an genetic algorithm embedded into a flocking simulation. The flocking itself has been derived from NCCA Embedded Python code, the processing.org flocking example and the YABI flocking example.

## Requirements:

This works best using virtualenv (https://virtualenv.pypa.io/en/stable/) to setup a stable python environment to run in.

- NCCA PyNGL (https://github.com/NCCA/NGL)
- PyQT5 (https://riverbankcomputing.com/software/pyqt/download5)
- numpy (https://docs.scipy.org/doc/numpy-dev/user/quickstart.html)
- OpenGL (https://www.opengl.org/sdk/)


## Running the system:

Make sure your Python environment is setup properly with the required packages.
Run NGLWindow.py.

## Using the system:

The GUI can be navigated using the mouse. The simulation cannot be interacted with once the start button has been clicked.
There are three tabs to flick between: Boids (to change boid parameters), Predators (to change predator parameters) and Genetics (to change genetic breeding parameters of the boids).

### Boids

Number of Boids: Adjust how many boids will appear in the simulation.

Cohesion Weight: Adjust the weighting by which the boids will cohese together in the flock.

Separation Weight: Adjust the weighting by which the boids will separate from each other within the flock.

Alighment Weight: Adjust the weighting by which the boids will align with each other in the flock.


### Predators

Number of Predators: Adjust how many predators will appear in the simulation.

Attack Weight: Adjust how aggressively the predators will try to seek out boids and move towards them

Sight Radius: Adjust how far the predators can see and which boids they will react to

Slowness: Adjust how slow the predators are, the higher the number the slower they become


### Genetics

Maximum Initial Awareness: Adjust the maximum possible range at which the boids will begin to notice predators close to them.

Maximum Initial Strength: Adjust the maximum possible level of strength the boids will have. The higher strength a boid has, the longer it can attempt to outrun a predator.

Maximum Initial Tiredness Rate: Adjust the maximum rate at which a boid tires whilst running away from a predator. The higher this number, the faster a boid will tire.

Maximum Initial Recovery Rate: Adjust the maximum rate at which a boid will recover energy after having become exhausted. The higher this number, the faster a boid will be able to recover energy.

