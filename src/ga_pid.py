import matplotlib.pyplot as plt
import numpy as np
import math
import sys
from pid import PID
from bicycle import Bicycle
import multiprocessing
import timeit

# TODO: develope a way to adapt mutation rate as the algorithm converges

class GA():
    def __init__(self, population, generations, bicycle, fault):
        self.bicycle = bicycle
        self.population_size = population
        self.population = np.empty((self.population_size), dtype=object)
        self.mutation_rate = 0.5
        self.crossover_point = 3
        self.chromosome_size = 6
        self.number_parents = int(math.sqrt(population))
        self.generations = generations
        self.eta = 5
        self.fittest = PID(bicycle.k)
        self.fittest.fitness = 1000000
        self.fit_norm = 0.0
        self.last_norm = 100000.0
        self.fit_plot = []
        self.fault = fault
        self.mu = np.array([10.0, 1.0, 0.1, 20.0, 1.0, 0.1])
        self.sigma = np.array([3.0, 3.0, 3.0, 3.0, 3.0, 3.0])

    def setup(self):
        for i in range(self.population_size):
            k = np.empty((self.chromosome_size))
            k[0] = np.random.uniform(low=0, high=1.0)
            k[1] = np.random.uniform(low=0, high=0.1)
            k[2] = np.random.uniform(low=0, high=0.01)
            k[3] = np.random.uniform(low=0, high=1.0)
            k[4] = np.random.uniform(low=0, high=0.1)
            k[5] = np.random.uniform(low=0, high=0.01)
            pid = PID(k)

            self.population[i] = pid

    def evolve(self):
        i = self.generations
        k = 0
        while i > 0 and self.fittest.fitness > self.eta:
            print("Generation #%s" % (self.generations - i + 1))
            print("calculate fitness")
            self.fitness()
            print("select parents")
            parents = self.selection()
            self.mu = self.fittest.k
            print(str(float(1.0/self.generations)))
            print(str(float(self.generations - k)))
            rate = (float(1.0/self.generations)*float(self.generations - k))
            print("Mutation Rate: %s" % rate)
            self.sigma = (3 * rate)*np.ones(6)
            print("New mu")
            print(self.mu)
            print("New sigma")
            print(self.sigma)
            print("crossover")
            self.crossover(parents)
            print("mutation")
            self.mutation()
            i = i - 1
            k += 1

        print("done evolving")
        return_dict = [0]
        self.fit_plot = np.array(self.fit_plot)
        t = np.arange(self.fit_plot.shape[0])
        plt.figure(figsize = (7,7))
        plt.plot(t, self.fit_plot, color='blue', linewidth=2)
        plt.xlim((0,25))
        plt.xlabel('generations')
        plt.ylabel('loss')
        plt.title('Genetic Algorithm Convergence')
        plt.show()
        print(self.fittest.k)
        self.bicycle.driveAlongPath(0, self.fittest, return_dict, 1, self.fault)

        return self.fittest.k

    def fitness(self):
        # start_time = timeit.default_timer()
        #
        # dict = [0]*self.population.shape[0]
        # for i in range(self.population.shape[0]):
        #     self.bicycle.driveAlongPath(i, self.population[i], dict, 0)
        #
        # elapsed = timeit.default_timer() - start_time
        #
        # print('Finished in %s second(s)' % round((elapsed), 3))
        #
        # for i in range(self.population.shape[0]):
        #     self.population[i].fitness = dict[i]

        start_time = timeit.default_timer()
        manager = multiprocessing.Manager()
        return_dict = manager.dict()
        processes = []
        for i in range(self.population.shape[0]):
            p = multiprocessing.Process(target=self.bicycle.driveAlongPath, args=[i, self.population[i], return_dict, 0, self.fault])
            p.start()
            processes.append(p)

        for process in processes:
            process.join()
        print("done calculating the fitness of this generation")

        elapsed = timeit.default_timer() - start_time

        print('Finished in %s second(s)' % round((elapsed), 3))

        for i in range(self.population.shape[0]):
            self.population[i].fitness = return_dict[i]

    def selection(self):
        print("\n")
        print("unsorted parents")
        parents = self.population[0:self.number_parents]
        fitness = []
        for parent in parents:
            fitness.append(parent.fitness)
            print(parent.fitness)
        self.quickSort(self.population, 0, self.population.shape[0]-1)
        print("\n")
        print("fittest parents")
        parents = self.population[0:self.number_parents]
        fitness = []
        for parent in parents:
            fitness.append(parent.fitness)
            print(parent.fitness)

        self.fittest = parents[0]
        self.fit_plot.append(self.fittest.fitness)
        fitness = np.array(fitness)
        self.fit_norm = np.linalg.norm(fitness)
        print("fitness norm")
        print(self.fit_norm)
        dt = self.fit_norm

        # if dt < 1000 and dt >= 500:
        #     self.mutation_rate = 0.75
        # elif dt < 500 and dt >= 200:
        #     self.mutation_rate = 0.5
        # elif dt < 200 and dt >= 100:
        #     self.mutation_rate = 0.35
        # elif dt < 100:
        #     self.mutation_rate = 0.25

        print("mutation rate: %s" % self.mutation_rate)
        self.last_norm = self.fit_norm

        return parents

    def crossover(self, parents):
        print(self.number_parents)
        new_population = parents
        new_population = np.reshape(new_population, [self.number_parents, 1])
        print(new_population.shape)

        for i in range(self.number_parents):
            for j in range(i+1, self.number_parents):
                parent1 = parents[i].k
                parent2 = parents[j].k
                offspring1 = [parent1[0], parent1[1], parent1[2], parent2[3], parent2[4], parent2[5]]
                offspring2 = [parent2[0], parent2[1], parent2[2], parent1[3], parent1[4], parent1[5]]
                offspring1 = np.array(offspring1)
                offspring2 = np.array(offspring2)
                offspring1_pid = PID(offspring1)
                offspring2_pid = PID(offspring2)
                new_offspring = np.empty([2,1], dtype=object)
                new_offspring[0,0] = offspring1_pid
                new_offspring[1,0] = offspring2_pid

                new_population = np.concatenate((new_population, new_offspring), axis=0)

        print(new_population.shape[0])
        for i in range(self.population.shape[0]):
            self.population[i].k = new_population[i,0].k

    def mutation(self):
        print("build mutation")
        for i in range(self.number_parents, self.population.shape[0]):
            if np.random.uniform(0,1) < self.mutation_rate:
                self.population[i].k[0] = math.fabs(np.random.normal(self.mu[0], self.sigma[0]))
            if np.random.uniform(0,1) < self.mutation_rate:
                self.population[i].k[1] = math.fabs(np.random.normal(self.mu[1], self.sigma[1]))
            if np.random.uniform(0,1) < self.mutation_rate:
                self.population[i].k[2] = math.fabs(np.random.normal(self.mu[2], self.sigma[2]))
            if np.random.uniform(0,1) < self.mutation_rate:
                self.population[i].k[3] = math.fabs(np.random.normal(self.mu[3], self.sigma[3]))
            if np.random.uniform(0,1) < self.mutation_rate:
                self.population[i].k[4] = math.fabs(np.random.normal(self.mu[4], self.sigma[4]))
            if np.random.uniform(0,1) < self.mutation_rate:
                self.population[i].k[5] = math.fabs(np.random.normal(self.mu[5], self.sigma[5]))


    def quickSort(self, x, start, end):
        if(start >= end):
            return
        index = self.partition(x, start, end)
        self.quickSort(x, start, index-1)
        self.quickSort(x, index + 1, end)

    def partition(self, x, start, end):
        pivot_index = start
        pivot_value = x[end].fitness
        for i in range(start, end+1):
            if x[i].fitness < pivot_value:
                self.swap(x, i, pivot_index)
                pivot_index = pivot_index + 1
        self.swap(x, pivot_index, end)

        return pivot_index

    def swap(self, x, a, b):
        temp = x[a]
        x[a] = x[b]
        x[b] = temp
