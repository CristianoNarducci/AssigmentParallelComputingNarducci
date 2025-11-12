# Boids Algorithm with OpenMP parallelization

Assignment project for **Parallel Computing** course, hosted by Prof. *Marco Bertini*.
  
Implemented cpu parallelization by OpenMP library and visualization of flocking objects by SMFL graphical Library.
The program compare two different type of structure passed at the threads for performance measurements:
* **Array of structures**: single array with every boids structure inside it
* **Structure of Arrays**: single structure with arrays of all position and velocity of boids

The code parallelize and implements the [algorithm](https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html#Pseudocode) created by *V. Hunter Adams* based from Boids program by Craig Reynolds

## Requirements
* **OpenMP** library installed
* CMake 4.0 or newer
* **SFML** library. In the CMake file it's present the code for automatically downloading and installing the library in project directory