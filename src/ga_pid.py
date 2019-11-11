import matplotlib.pyplot as plt
import numpy as np
import math
import sys
from pid import PID
from bicycle import Bicycle

class GA():
    def __init__(self, population, mutation, generations, bicycle):
        self.bicycle = bicycle
        self.population_size = population
        self.population = np.empty((self.population_size), dtype=object)
        self.mutation_rate = mutation
        self.crossover_point = 3
        self.chromosome_size = 6
        self.generations = generations

    def setup(self):
        for i in range(self.population_size):
            k = np.empty((self.chromosome_size))
            k[0] = np.random.uniform(low=0, high=2)
            k[1] = np.random.uniform(low=0, high=0.1)
            k[2] = np.random.uniform(low=0, high=0.1)
            k[3] = np.random.uniform(low=0, high=10)
            k[4] = np.random.uniform(low=0, high=0.1)
            k[5] = np.random.uniform(low=0, high=0.1)
            pid = PID(k)

            self.population[i] = pid

    def evolve(self):
        # TODO: write selection algorithm to choose the most fit chromosomes
        # TODO: write a crossover algorithm
        # TODO: write a mutation algorithm
        # TODO: write exit condition algorithm
        
        self.fitness()

    def fitness(self):
        for chromosome in self.population:
            self.bicycle.pid = chromosome
            self.bicycle.driveAlongPath()
            chromosome.fitness = self.bicycle.objective()
            # print("chromosome k")
            # print(chromosome.k)
            # print(chromosome.fitness)
        print("done calculating the fitness of this generation")
