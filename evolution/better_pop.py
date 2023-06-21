from __future__ import print_function
from neat.population import Population, CompleteExtinctionException
from neat.six_util import iteritems, itervalues

try:
    import cPickle as pickle  # pylint: disable=import-error
except ImportError:
    import pickle  # pylint: disable=import-error


class better_Population(Population):
    def __init__(self, config, initial_state=None):
        super().__init__(config, initial_state)

    def run(self, fitness_function, n=None, reeval_first_gen=False):
        """
        reorders Population's run method so that saving happens before remaking a population

        @param reeval_first_gen: whether to set all fitness values to None initially
        """
        if self.config.no_fitness_termination and (n is None):
            raise RuntimeError("Cannot have no generational limit with no fitness termination")
        if reeval_first_gen:
            for g in self.population:
                self.population[g].fitness = None

        k = 0
        while n is None or k < n:
            k += 1

            self.reporters.start_generation(self.generation)

            # Evaluate all genomes using the user-provided function.
            fitness_function(list(iteritems(self.population)), self.config)

            # Gather and report statistics.
            best = None
            for g in itervalues(self.population):
                if best is None or g.fitness > best.fitness:
                    best = g
            self.reporters.post_evaluate(self.config, self.population, self.species, best)

            # Track the best genome ever seen.
            if self.best_genome is None or best.fitness > self.best_genome.fitness:
                self.best_genome = best

            if not self.config.no_fitness_termination:
                # End if the fitness threshold is reached.
                fv = self.fitness_criterion(g.fitness for g in itervalues(self.population))
                if fv >= self.config.fitness_threshold:
                    self.reporters.found_solution(self.config, self.generation, best)
                    break

            # reorder here, so that loading gen k will have all fitness values already loaded
            self.reporters.end_generation(self.config, self.population, self.species)

            # Create the next generation from the current generation.
            self.population = self.reproduction.reproduce(self.config, self.species,
                                                          self.config.pop_size, self.generation)

            # Check for complete extinction.
            if not self.species.species:
                self.reporters.complete_extinction()

                # If requested by the user, create a completely new population,
                # otherwise raise an exception.
                if self.config.reset_on_extinction:
                    self.population = self.reproduction.create_new(self.config.genome_type,
                                                                   self.config.genome_config,
                                                                   self.config.pop_size)
                else:
                    raise CompleteExtinctionException()

            # Divide the new population into species.
            self.species.speciate(self.config, self.population, self.generation)

            self.generation += 1

        if self.config.no_fitness_termination:
            self.reporters.found_solution(self.config, self.generation, self.best_genome)

        return self.best_genome
