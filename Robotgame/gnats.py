import rg
import time
import math
from collections import deque

'''
    Gnats

    Fly around attacking target once then fleeing to another target

    Todo
    - Don't flee if you can win combat
    - Don't all mob the same enemy, only send 2 or 3 bots at each mob. Use a scheduler that runs
        at the beginning of a new turn
'''

# Config constants
FLEE_HP_THRESHHOLD = 10
# Time in ms before we warn how long a robot's act method took
ACT_TIMER_WARNING_THRESHHOLD = 200
# Robots will assemble into groups of this size for protection
GROUP_SIZE = 2

# When fleeing, how much do we prefer a closer location to a safer location.  If set to 0, we will always
# run to the location furthest from enemies.  If set to 1, we will value one movement as much as an enemy
# being one tile further away from us.  If set to greater than one, we'd rather stay where we are.
FLEE_DISTANCE_COST = 0.2

class Robot:
    state = None
    attack_count = 0
    guard_count = 0
    move_count = 0
    times = []
    def act(self, game):
        start = time.clock()

        if Robot.state == None:
            Robot.state = RobotsState(self.player_id)

        if (Robot.state.check_is_new_turn(game.get('turn'))):
            Robot.state.init_new_turn(game)

        Robot.state.robot_number += 1

        move = Robot.state.generate_move(self)

        if move[0] == "guard":
            Robot.guard_count += 1
        elif move[0] == "attack":
            Robot.attack_count += 1
        elif move[0] == "move":
            Robot.move_count += 1
        

        finish = time.clock()
        if ((finish-start) * 1000) > ACT_TIMER_WARNING_THRESHHOLD:
            print "Warning: act took " + `(finish-start) * 1000` + "ms"

        Robot.times.append((finish-start) * 1000)

        if len(Robot.state.friendly_robots) == Robot.state.robot_number and game.get('turn') == rg.settings.max_turns - 1:
            print "Summary:"
            print "Attacks: " + `Robot.attack_count` + " Moves: " + `Robot.move_count` + " Guards: " + `Robot.guard_count`
            Robot.attack_count = 0
            Robot.guard_count = 0
            Robot.move_count = 0
            print "Turn times"
            print "Min: " + `min(Robot.times)` + "ms"
            print "Max: " + `max(Robot.times)` + "ms"
            print "Average: " + `sum(Robot.times)/len(Robot.times)` + "ms"
        return move

class RobotsState:
    def __init__(self, friendly_id):
        start = time.clock()
        # play_area_locations is a set of location tuples that are in the play area (not invalid and not obstructions)
        # This set is pre-initialized on startup and used in various graph operations that iterate over the gameplay space
        self.play_area_locations = set()
        self._init_graph()
        finish = time.clock()
        self.turn = 0
        self.friendly_id = friendly_id
        self.enemy_id = 0 if friendly_id == 1 else 1

        # For cooperative path-finding, this dict maps a locaton to a number representing number
        # of turns in the future that the tile is requested use for.  It's first-come-first-served,
        # so for generating critical paths (such as fleeing injured robots), they should be handled
        # on an emergency basis by the first robot of the turn
        self.future_locations = set()
        self.enemy_robots = set()
        self.friendly_robots = set()
        self.flee_graph = dict()
        print "Graph setup done in " + `(finish-start) * 1000` + "ms"

    def _init_graph(self):
        '''
        Populates a graph of locations reachable from a given location which excludes illegal
        spaces such as invalid, spawn, and obstacle.
        The graph is implemented as a simple dictionary of location => list of locations
        '''
        # Set up play_area_locations
        self.play_area_locations = set()

        for x in range(rg.settings.board_size):
            for y in range(rg.settings.board_size):
                if rg.loc_types((x,y)).isdisjoint(['invalid', 'obstacle']):
                    self.play_area_locations.add((x,y))

        self.graph = dict()
        for loc in self.play_area_locations:
            locs = rg.locs_around(loc, filter_out=('invalid', 'obstacle'))
            self.graph[loc] = locs

    def _init_flee_graph(self):
        '''
        Populates a graph of locations reachable from a given location which excludes spaces with
        an enemy in or adjacent to
        '''
        self.flee_graph.clear()

        # Build a set of locations with enemies on or adjacent to them
        bad_locations = set()
        for i in self.enemy_robots:
            bad_locations.add(i.location)
            for j in rg.locs_around(i.location, filter_out=('invalid', 'obstacle')):
                bad_locations.add(j)

        for x in range(rg.settings.board_size):
            for y in range(rg.settings.board_size):
                locs = set(rg.locs_around((x,y), filter_out=('invalid', 'obstacle', 'spawn')))
                # Only add locs that don't have an enemy on them or adjacent to them
                self.flee_graph[(x,y)] = list(locs - bad_locations)

    def check_is_new_turn(self, turn):
        '''
        Compares the stored turn value with the provided one and if they are different, updates the instance
        variable and returns true.  This allows us to handle certain once-per-turn tasks such as updating 
        robot to id map
        '''
        if turn != self.turn:
            if turn != self.turn + 1 and self.turn != 99:
                print "Turns did not increment consecutively - bug? self.turn: " + `self.turn` + "; turn: " + `turn`
            self.turn = turn
            return True
        return False

    def init_new_turn(self, game):
        '''
        Resets data structures that are only valid for a single turn - typically friendly and enemy locations
        '''
        self.future_locations = set()
        self.game = game
        self.enemy_robots.clear()
        self.friendly_robots.clear()
        for loc, bot in self.game.get('robots').items():
            newBot = KnownRobot(bot)
            if self._is_enemy(bot.get('player_id')):
                self.enemy_robots.add(newBot)
            else:
                self.friendly_robots.add(newBot)
        self._init_flee_graph()
        self.robot_number = 0
        
    def generate_move(self, robot):
        '''
        Entry-point for turn generation
        '''
        if robot.hp <= FLEE_HP_THRESHHOLD:
            return self._flee(robot)

        if Utils.is_spawn(robot.location) and Utils.new_robots_spawning(self.turn + 1):
            for loc in rg.locs_around(robot.location, filter_out=('invalid', 'obstacle', 'spawn')):
                return ['move', loc]

        enemyId = 0 if robot.player_id == 1 else 1
        closestEnemy = self._closest_robot(enemyId, robot.location)

        if closestEnemy != None:
            if rg.wdist(robot.location, closestEnemy.location) <= 1:
                return ['attack', closestEnemy.location]
            else:
                path = self._cooperative_shortest_path(self.graph, robot.location, closestEnemy.location)
                if path == None:
                    return ['guard']
                else:
                    self._commit_path_to_future_locations(path)
                    return ['move', path[1]]

        # move toward the center
        path = self._cooperative_shortest_path(self.graph, robot.location, rg.CENTER_POINT)
        if path == None:
            return ['move', rg.toward(robot.location, rg.CENTER_POINT)]
        else:
            self._commit_path_to_future_locations(path)
            return ['move', path[1]]

    # Path-finding helpers that use the BFS implementation further below.  The BFS algorithm I'm using has
    # been altered to take a callback with a location and route depth.  This is looked up in the
    # cooperative path-finding set to determine if another both already intends to move there next turn
    def _cooperative_shortest_path(self, graph, start, end):
        return shortest_path(graph, start, end, lambda loc, n: self._coop_callback(loc, n))

    def _uncooperative_shortest_path(self, graph, start, end):
        return shortest_path(graph, start, end, lambda loc, n: True)

    def _coop_callback(self, location, n):
        if (location, n) in self.future_locations:
            return False

        # Try to add some more smarts.  If there's a friendly bot there right now and it has an enemy
        # next to it, assume it'll stay there attacking
        closest_friendly = self._closest_robot(self.friendly_id, location, False)
        
        if closest_friendly != None and closest_friendly.location == location:
            closestEnemy = self._closest_robot(self.enemy_id, location)
            if closestEnemy == None:
                # There are no enemies present (unlikely), assume robot isn't going to move since there's 
                # nothing to attack
                return False
            else:
                if rg.wdist(location, closestEnemy.location) <= 1 and closest_friendly.hp > max(rg.settings.attack_range):
                    return False

        # Don't path to a spawn location if spawns are happening next turn
        if Utils.is_spawn(location) and Utils.new_robots_spawning(self.turn + 1):
            return False

        return True

    def _commit_path_to_future_locations(self, path):
        n = 0
        for loc in path:
            self.future_locations.add((loc, n))
            n += 1

    def _flee(self, robot):
        '''
        Bravely run away!
        If there is a nice chunk of empty space to move to close-by, run there.
        If not, run to the protection of a nearby friendly

        '''
        # Find the closest point that's the farthest from enemies

        # distance_cost is how much we subtract from a tile's enemy farness to factor in that we'd rather be lazy
        # and not move if we don't have to.
        distance_cost = FLEE_DISTANCE_COST

        tile_scores = dict()
        for loc in self.play_area_locations:
            distance_to_tile = rg.wdist(robot.location, loc)
            closest_enemy = self._closest_robot(self.enemy_id, loc)
            distance_to_closest_enemy = 0
            if closest_enemy == None:
                # No enemies left - move to center since that'll be far from new enemy spawns
                return ['move', rg.toward(robot.location, rg.CENTER_POINT)]
            else:
                distance_to_closest_enemy = rg.wdist(loc, closest_enemy.location)

            tile_score = int(distance_to_closest_enemy - (distance_to_tile * distance_cost))
            tile_scores[loc] = tile_score

        # How many of the top locations we will try pathing to incase the best option is unreachable
        flee_attempts = 5
        for flee_location in sorted(tile_scores, key=lambda a: tile_scores[a], reverse=True):
            if (robot.location == flee_location):
                # Don't bother pathing since our current location is the best
                return ['guard']

            path = self._cooperative_shortest_path(self.flee_graph, robot.location, flee_location)
            if path == None:
                flee_attempts -= 1
                if flee_attempts <= 0:
                    break;
                else:
                    continue
            else:
                self._commit_path_to_future_locations(path)
                return ['move', path[1]]
        
        closestEnemy = self._closest_robot(self.enemy_id, robot.location)
        if closestEnemy != None:
            if rg.wdist(robot.location, closestEnemy.location) <= 1:
                return ['attack', closestEnemy.location]
        
        # We can't path to safety but we're not next to an enemy so stay where we are
        return ['guard']

    # Given a location and an id, finds the closest robot of that type
    def _closest_robot(self, id, location, exclude_location = True):
        closest = None
        closestDistance = 300

        robots = set()
        if id == self.friendly_id:
            robots = self.friendly_robots
        else:
            robots = self.enemy_robots

        for bot in robots:
            if rg.wdist(location, bot.location) < closestDistance and ((not exclude_location) or (bot.location != location)):
                closest = bot
                closestDistance = rg.wdist(location, bot.location)

        return closest

    def _generate_bfs_path(self, start, destination):
        return None

    def _is_friendly(self, id):
        return id == self.friendly_id

    def _is_enemy(self, id):
        return id != self.friendly_id

# Not sure if we want to maintain a constant idea of all robots and their specific instances
class KnownRobot:
    next_id = 1
    def __init__(self, robot):
        self.id = KnownRobot.next_id
        KnownRobot.next_id += 1
        self.location = robot.get('location')
        self.hp = robot.get('hp')

class Utils:
    @staticmethod
    def is_spawn(location):
        return 'spawn' in rg.loc_types(location)

    @staticmethod
    def new_robots_spawning(turn):
        return turn % 10 == 1

    @staticmethod
    def print_board(data):
        '''
        Prints a textual display of the play area with coordinates and for each tile, reads
        from the dictionary 'data' which must be have a location tuple key: (x, y) => string / int
        '''
        for y in range(rg.settings.board_size + 1):
            line = ""
            for x in range(rg.settings.board_size + 1):
                if y == 0:
                    if x == 0:
                        line += "   "
                    else:
                        line += '{:>3}'.format(`x-1`)
                else:
                    if x == 0:
                        line += '{:>3}'.format(`y-1`)
                    else:
                        if (x-1,y-1) in data:
                            line += '{:>3}'.format(`data[(x-1,y-1)]`)
                        else:
                            line += "   "
            print line


def bfs(g, start, cooperative_function):
    queue = deque([((None, -1), (start, 0))])
    visited = set([start])
    while queue:
        parent, n = queue.popleft()
        parent, parent_distance = parent
        n, n_distance = n
        yield parent, n
        new = set(g[n]) - visited
        visited |= new
        for child in new:
            if cooperative_function(child, n_distance + 1):
                queue.extend([((n, n_distance), (child, n_distance + 1))])

def shortest_path(g, start, end, cooperative_function):
    parents = {}
    for parent, child in bfs(g, start, cooperative_function):
        parents[child] = parent
        if child == end:
            revpath = [end]
            while True:
                parent = parents[child]
                revpath.append(parent)
                if parent == start:
                    break
                child = parent
            return list(reversed(revpath))
    return None # or raise appropriate exception
