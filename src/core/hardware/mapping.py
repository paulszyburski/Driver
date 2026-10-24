class Mapping:
    def __init__(self):
        self.free_space = set()
        self.boundaries = set()
        self.obstacles = set()
        self.goal = set()

    @staticmethod
    def _position(x, y):
        # Store map positions with 1 cm resolution.
        return round(x), round(y)

    def add_free_space(self, x, y):
        position = self._position(x, y)

        if position not in self.boundaries and position not in self.obstacles:
            self.free_space.add(position)

    def add_boundary(self, x, y):
        position = self._position(x, y)

        self.boundaries.add(position)
        self.free_space.discard(position)

    def add_obstacle(self, x, y):
        position = self._position(x, y)

        self.obstacles.add(position)
        self.free_space.discard(position)

    def add_goal(self, x, y):
        self.goal.add(self._position(x, y))
