"""\
Particle Swarm Optimization

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import numpy
from random import uniform


class Particle(numpy.ndarray):
    _gbest = None
    _gbest_fitness = None

    def __init__(self, *args):
        self.velocity = numpy.ndarray(self.shape[0])
        self._best = None
        self._best_fitness = None
        self.neighborhood = set()

    @property
    def best(self):
        return (self._best, self._best_fitness)

    @property
    def nbest(self):
        if not self.neighborhood:
            return self.gbest
        candidates = [particle.best for particle in self.neighborhood]
        return max(candidates, key=lambda best: best[1])

    @property
    def gbest(self):
        return (self.__class__._gbest, self.__class__._gbest_fitness)

    def initialize(self, bounds, l=1.0):
        for d in range(self.shape[0]):
            self[d] = uniform(bounds[d][0], bounds[d][1])
            span = bounds[d][1] - bounds[d][0]
            vmax = l * span
            self.velocity[d] = min(max(uniform(-span, span), -vmax), vmax)

    def update(self, omega, phip, phin, constraint, bounds, l=1.0):
        for d in range(self.shape[0]):
            vmax = l * (bounds[d][1] - bounds[d][0])
            self.velocity[d] = min(max(omega * self.velocity[d] \
                + uniform(0, phip) * (self.best[0][d] - self[d]) \
                + uniform(0, phin) * (self.nbest[0][d] - self[d]), -vmax), vmax)
        self += self.velocity
        constraint(self, bounds)

    def update_best(self, fitness):
        if fitness > self._best_fitness:
            self._best = tuple(self)
            self._best_fitness = fitness
            if fitness > self.__class__._gbest_fitness:
                self.__class__._gbest = tuple(self)
                self.__class__._gbest_fitness = fitness
    
    def norm_dist_to_gbest(self, bounds):
        base = numpy.array([b[0] for b in bounds])
        span = numpy.array([b[1] for b in bounds]) - base
        center = (numpy.array(self.gbest[0]) - base) / span
        norm = (self - base) / span
        return numpy.linalg.norm(norm - center)


topologies = {None: lambda particles: None}

def topology(f):
    topologies[f.__name__] = f
    return f

@topology
def ring(particles):
    for i, particle in enumerate(particles):
        particle.neighborhood.add(particle)
        particle.neighborhood.add(particles[i - 1])
        particle.neighborhood.add(particles[(i + 1) % len(particles)])

@topology
def star(particles):
    for particle in particles[1:]:
        particle.neighborhood.add(particle)
        particle.neighborhood.add(particles[0])


constraints = {None: lambda particle, bounds: None}

def constraint(f):
    constraints[f.__name__] = f
    return f

@constraint
def nearest(particle, bounds):
    for d in range(particle.shape[0]):
        particle[d] = max(particle[d], bounds[d][0])
        particle[d] = min(particle[d], bounds[d][1])

@constraint
def random(particle, bounds):
    for d in range(particle.shape[0]):
        if particle[d] < bounds[d][0] or particle[d] > bounds[d][1]:
            particle[d] = uniform(bounds[d][0], bounds[d][1])


def particle_swarm_optimize(fitness, dimension, bounds, size, omega, phip, phin,
                            clamp=1.0, it=None, af=float('inf'), cluster=(1.0,
                            0.0), topology_type=None, constraint_type=None):
    particles = [Particle(dimension) for i in range(size)]
    topologies[topology_type](particles)
    for particle in particles:
        particle.initialize(bounds, l=clamp)
    i = 0
    while not it or i < it:
        for particle in particles:
            particle.update_best(fitness(particle))
        yield particles[0].gbest
        if particles[0].gbest[1] >= af and \
            (particles[0].gbest[2] == -float('inf') \
             or particles[0].gbest[2] >= af):
            break
        if sum([particle.norm_dist_to_gbest(bounds) < cluster[1] \
            for particle in particles]) >= int(cluster[0] * size):
            break
        for particle in particles:
            particle.update(omega, phip, phin, constraints[constraint_type],
                            bounds)
        i += 1
