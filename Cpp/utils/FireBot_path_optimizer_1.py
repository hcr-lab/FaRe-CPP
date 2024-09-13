import random


class WayPointOptimizer:
    def __init__(self, goals, alpha, max_iterations):
        self.goals = goals
        self.alpha = alpha or 0.3
        self.max_iterations = max_iterations or 5
        self.best_solution = None
        self.best_cost = float('inf')

    @staticmethod
    def calculate_distance(point1, point2):
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

    def greedy_randomized_construction(self):
        current_position = self.goals[0]
        solution = [current_position]
        remaining_goals = self.goals[1:]

        while remaining_goals:
            distances = [(self.calculate_distance(current_position, goal), goal) for goal in remaining_goals]
            distances.sort()
            top_candidates = distances[:max(1, int(self.alpha * len(distances)))]
            _, next_goal = random.choice(top_candidates)
            solution.append(next_goal)
            remaining_goals.remove(next_goal)
            current_position = next_goal

        return solution

    @staticmethod
    def two_opt_swap(route, i, k):
        new_route = route[0:i]
        new_route.extend(reversed(route[i:k + 1]))
        new_route.extend(route[k + 1:])
        return new_route

    def calculate_total_distance(self, route):
        return sum(self.calculate_distance(route[i], route[i+1]) for i in range(len(route) - 1))

    def local_search(self, solution):
        best_route = solution
        best_cost = self.calculate_total_distance(solution)

        improved = True
        while improved:
            improved = False
            for i in range(1, len(best_route) - 2):
                for k in range(i + 1, len(best_route)):
                    new_route = self.two_opt_swap(best_route, i, k)
                    new_cost = self.calculate_total_distance(new_route)
                    if new_cost < best_cost:
                        best_route = new_route
                        best_cost = new_cost
                        improved = True
            if not improved:
                break

        return best_route

    def run(self):
        for _ in range(self.max_iterations):
            solution = self.greedy_randomized_construction()
            solution = self.local_search(solution)
            cost = self.calculate_total_distance(solution)

            if cost < self.best_cost:
                self.best_solution = solution
                self.best_cost = cost

        return self.best_solution
