# 3D Fluid Simulation With User Interaction

## Function

This program simulates a simple fluid using the SPH method. The user can interact with the fluid using the mouse.

## Detailed Assignment

The assignment was to create a fluid simulation in Unity which could be interacted with and whose parameters (gravity, viscosity, particle count ...) could be changed

## Algorithm

The algorithm used for the simulation is SPH (Smoothed Particle Hydrodynamics). This algorithm discerizes the fluid into particles whose influence is smoothed out in their radius. The fluid behaviour emerges from pairwise interactions between all the particles. The influence each particle has on another particles density is determined by a smoothing function (in this case the Gaussian smoothing function) and the influence on acceleration is determined by the derivative of this smoothing function.

In each timestep we calculate the acceleration of each particle based on a chosen fluid dynamics equation (Navier-Stokes equations or in this case the Euler equation for an ideal fluid) and from this acceleration we calculate the new position.

## Program

### User

The user can interact with the fluid via 2 actions:

- left mouse button press and drag: makes the particles within the interaction radius of the mouse position attracted to that position
- right mouse button press and drag: makes the particles withing the interaction radius of the mouse position reprelled from that position

and change the fluids parameters with sliders:

- Particle Count
- Gravity
- Viscosity

### SPHFluid.cs

Contains all the logic for the simulation:

- Start(): initializes particles, gradient, main camera for mouse interactions and the particle mass
- Update(): handles mouse interactions and updates all particles
- HandleMouseInput(): checks mouse interactions and calculates world position of mouse using the main camera
- InitializeParticles(): places all particles in random positions in the border box and gives them a velocity with a random direction
- UpdateParticles(float dt): updates particle positions based on computed accelerations, checks collisions and updates particle colors based on their speed
- CheckCollisions(int index): checks collisions of particle with index 'index' and updates its position and velocity if it is colliding
- struct ComputeDensityPressureJob: computes densities and pressures using the Job system and BurstCompile for paralellization
  - GaussianSmoothingKernel(float r, float h): computes the effect a particle at distance r has on the density and pressure of the current particle
- struct ComputeAccelerationJob: computes accelerations in parallel and adds an interaction force if a mouse interaction is detected and the particle is in the interaction radius
  - GaussianSmoothingKernelDerivative(float3 difference, float distance, float smoothingLength): calculates the vector of the force by which a particle at the relative location 'difference' is acting on the current particle
- UpdateParticleCount(), UpdateViscosity() and UpdateGravity(): handler functions for event listeners listening to changes to the corresponding sliders in the ui
- Play(): handler function for the press event of the play button. Starts the simulation and updates the ui so the user can change the Viscosity and Gravity with sliders and adds a Restart button to restart the simulation.
- Restart(): handler function for the press event of the restart button. Restarts the whole scene, removing the Viscosity and Gravity sliders and showing the Particle Count slider and Play button.

## Versions and libraries

Unity version: 2022.3.8f1
Downloaded Unity Libraries: Collections, Jobs, Burst, Mathematics

## Process

My assignment choice was inspired by Sebastian Lagues video on YouTube ([Coding Adventure: Simulating Fluids](https://www.youtube.com/watch?v=rSKMYc1CQHE&t=412s)). At first I created a whole program based on his approach but the simulation was not really behaving like a fluid and I could not figure out what was the problem so I had to start from scratch. I found an [article](https://philip-mocz.medium.com/create-your-own-smoothed-particle-hydrodynamics-simulation-with-python-76e1cec505f1) with a slightly different approach using the Euler equation for an ideal fluid and written in python. I created a Unity project based on this article and this time it looked a lot more promising and I made it 3D. I added some optimizations like using the Unity Jobs system and BurstCompile for the density, pressure and acceleration calculations as they can be parelellized for all the particles. I added the mouse interaction and the gradient for the particle color.

## Unfinished work

The program could be improved by implementing a grid-based neighbor search using hashing for searching the particles that are only close by and avoiding having to calculate the interactions between all pairs of particles. Furthermore the rendering of the particles could be done with GPU mesh instancing to improve the simulations performance.
