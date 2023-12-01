from pddl_parser.PDDL import PDDL_Parser

class Planner:

    # -----------------------------------------------
    # Solve FF (CUSTOM PLANNER)
    # -----------------------------------------------

    def calculate_hff(self, state, goal, ground_actions):
        hff = 0
        new_state = state
        while not goal.issubset(new_state):
            for act in ground_actions:
                if act.positive_preconditions.issubset(new_state):
                    new_state = new_state.union(act.add_effects) # Just compute the union, since no delete effects are considered
            hff += 1
        return hff

    def solve_ff(self, domain, problem):

        # Parser
        parser = PDDL_Parser()
        parser.parse_domain(domain)
        parser.parse_problem(problem)

        # Parsed data
        state = parser.state
        goal = parser.positive_goals

        # Do nothing if already solved
        if goal.issubset(state): return []

        # Grounding process (enumerating all possible actions in the problem)
        ground_actions = []
        for action in parser.actions:
            for act in action.groundify(parser.objects, parser.types):
                ground_actions.append(act)
        
        # Search
        plan = []
        while not goal.issubset(state):
            curr_hff = self.calculate_hff(state, goal, ground_actions)

            # Greedy search
            found_better = False
            for act in ground_actions:
                if act.positive_preconditions.issubset(state):
                    new_state = self.apply(state, act.add_effects, act.del_effects)
                    if self.calculate_hff(new_state, goal, ground_actions) < curr_hff:
                        state = new_state
                        plan.append(act)
                        found_better = True
                        break
            if found_better: continue

            # If the code reaches here, it means there is a plateau -> use BFS to continue
            visited = set([state])
            queue = [(state, [])]
            found_better = False
            while queue and not found_better:
                q_state, q_plan = queue.pop(0)
                for act in ground_actions:
                    if act.positive_preconditions.issubset(q_state):
                        new_state = self.apply(q_state, act.add_effects, act.del_effects)
                        if new_state not in visited:
                            if self.calculate_hff(new_state, goal, ground_actions) < curr_hff:
                                plan.extend(q_plan + [act])
                                state = new_state
                                found_better = True
                                break
                            visited.add(new_state)
                            queue.append((new_state, q_plan + [act]))

            # If even BFS couldn't find a node with a lower hff, then no plan exists
            if not found_better: return None
             
        return plan

    # -----------------------------------------------
    # Solve BFS (EXAMPLE PLANNER)
    # -----------------------------------------------

    def solve_bfs(self, domain, problem):

        # Parser
        parser = PDDL_Parser()
        parser.parse_domain(domain)
        parser.parse_problem(problem)

        # Parsed data
        state = parser.state
        goal_pos = parser.positive_goals
        goal_not = parser.negative_goals

        # Do nothing
        if self.applicable(state, goal_pos, goal_not):
            return []

        # Grounding process
        ground_actions = []
        for action in parser.actions:
            for act in action.groundify(parser.objects, parser.types):
                ground_actions.append(act)

        # Search
        visited = set([state])
        fringe = [state, None]
        while fringe:
            state = fringe.pop(0)
            plan = fringe.pop(0)
            for act in ground_actions:
                if self.applicable(state, act.positive_preconditions, act.negative_preconditions):
                    new_state = self.apply(state, act.add_effects, act.del_effects)
                    if new_state not in visited:
                        if self.applicable(new_state, goal_pos, goal_not):
                            full_plan = [act]
                            while plan:
                                act, plan = plan
                                full_plan.insert(0, act)
                            return full_plan
                        visited.add(new_state)
                        fringe.append(new_state)
                        fringe.append((act, plan))
        return None

    # -----------------------------------------------
    # Helper functions
    # -----------------------------------------------

    def applicable(self, state, positive, negative):
        return positive.issubset(state) and negative.isdisjoint(state)

    def apply(self, state, positive, negative):
        return state.difference(negative).union(positive)

# -----------------------------------------------
# Main
# -----------------------------------------------
if __name__ == '__main__':
    import time
    start_time = time.time()
    domain = "pddl/domain.pddl"
    problem = "pddl/problem.pddl"
    verbose = True
    planner = Planner()
    plan = planner.solve_ff(domain, problem)
    #plan = planner.solve_bfs(domain, problem)
    print('Time: ' + str(time.time() - start_time) + 's')
    if type(plan) is list:
        print('plan:')
        for act in plan:
            print(act if verbose else act.name + ' ' + ' '.join(act.parameters))
    else:
        print('No plan was found')
        exit(1)
