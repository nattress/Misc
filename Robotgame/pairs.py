import rg
import time
import math

# Data structures
#
# game
#  turn - int 0-100
#  robots
#   [coords] : { 
#                'player_id' : id,
#                'hp' : hp,
#                'location' : [coords]
#                }

'''
    Gnats

    Fly around attacking target once then fleeing to another target
'''

# Config constants
FLEE_HP_THRESHHOLD = 10
# Time in ms before we warn how long a robot's act method took
ACT_TIMER_WARNING_THRESHHOLD = 200
# Robots will assemble into groups of this size for protection
GROUP_SIZE = 2

class Robot:
    state = None
    def act(self, game):
        start = time.clock()

        if Robot.state == None:
            Robot.state = RobotsState(self.player_id)

        if (Robot.state.check_is_new_turn(game.get('turn'))):
            Robot.state.update_robot_ids(Utils.new_robots_spawning(game.get('turn')), game.get('robots'))
            Robot.state.init_new_turn(game)

        move = Robot.state.generate_move(self)

        finish = time.clock()
        if ((finish-start) * 1000) > ACT_TIMER_WARNING_THRESHHOLD:
            print "Warning: act took " + `(finish-start) * 1000` + "ms"

        return move

class RobotsState:
    def __init__(self, friendly_id):
        start = time.clock()
        self._init_graph()
        finish = time.clock()
        self.turn = 0
        self.friendly_id = friendly_id
        # For cooperative path-finding, this dict maps a locaton to a number representing number
        # of turns in the future that the tile is requested use for.  It's first-come-first-served,
        # so for generating critical paths (such as fleeing injured robots), they should be handled
        # on an emergency basis by the first robot of the turn
        self.future_locations = dict()
        self.enemy_robots = set()
        self.friendly_robots = set()
        print "Graph setup done in " + `(finish-start) * 1000` + "ms"

    def _init_graph(self):
        '''
        Populates a graph of locations reachable from a given location which excludes illegal
        spaces such as invalid, spawn, and obstacle.  Spawn tiles aren't exactly illegal, but
        until we're smart enough to avoid them only on spawn turns we should stay away.
        The graph is implemented as a simple dictionary of location => list of locations
        '''
        self.graph = dict()
        for x in range(rg.settings.board_size):
            for y in range(rg.settings.board_size):
                locs = rg.locs_around((x,y), filter_out=('invalid', 'spawn', 'obstacle'))
                self.graph[(x,y)] = locs
        
    def check_is_new_turn(self, turn):
        '''
        Compares the stored turn value with the provided one and if they are different, updates the instance
        variable and returns true.  This allows us to handle certain once-per-turn tasks such as updating 
        robot to id map
        '''
        if turn != self.turn:
            if turn != self.turn + 1:
                print "Turns did not increment consecutively - bug? self.turn: " + `self.turn` + "; turn: " + `turn`
            self.turn = turn
            return True
        return False

    def update_robot_ids(self, new_robots_spawned, robots):
        '''
        Called at the beginning of each turn to calculate the current location of each robot
        so we can map them all back to their IDs. For each robot, we recorded its planned new
        location if it intended to move.
        - We look at the intended location, and if we see a friendly robot there, we know it must be
            the same robot (we never knowingly send two friendly robots to the same tile)
        - If the intended location is empty, we must have moved there and gotten destroyed, or an
            enemy also tried to move there and we collided (and we lost 5 health)
            (or we have a bug in our routing logic and sent two robots to collide)
        - If the intended location is occupied by an enemy, we should never have tried to move there
            and we caused a collision (this would be a bug)
        - If we tried to move into a space previously occupied by a friendly robot, we need to resolve
            its move first.  This robot can only have moved if and only if the one infront of it moved.
            It's possible there is a cycle here if we were moving in a closed loop, too.
        '''
        #if new_robots_spawned:
            # Remove all friendly robots that were in spawn locations. This shouldn't happen with a good AI
            # unless we got trapped there by the enemy somehow
            # This must be done after resolving moves incase a friendly *did* want to move out of spawn but 
            # got blocked for some reason

        return None

    def init_new_turn(self, game):
        self.future_locations = dict()
        self.game = game
        self.enemy_robots.clear()
        self.friendly_robots.clear()
        for loc, bot in self.game.get('robots').items():
            newBot = KnownRobot(bot)
            if self._is_enemy(bot.get('player_id')):
                self.enemy_robots.add(newBot)
            else:
                self.friendly_robots.add(newBot)

        groups = self._group_robots()
        
        # Determine a set of action priorities for each robot ahead of time
        # If un-grouped, meet up
        # If adjacent to an enemy and out-numbered, flee
        # If grouped, find nearest fight, preferring fights where we outnumber the enemy

        # Priority list of actions
        # - Make sure all friendlies are paired up and if not schedule them to assemble
        # - Look for robots near death and have them flee
        # - Look for robots alone in a fight that they are going to lose
    def generate_move(self, robot):
        # Priority list of actions
        # - Flee if out-gunned towards a friendly group
        # - Pair up with another robot to coordinate attacks
        # - 

        #if self.hp <= FLEE_HP_THRESHHOLD:

        enemyId = 0 if robot.player_id == 1 else 1
        closestEnemy = self._closest_robot(enemyId, robot.location)

        if rg.wdist(robot.location, closestEnemy.location) <= 1:
            #awayLocs = rg.locs_around(location, filter_out=('invalid', 'obstacle'))
            return ['attack', closestEnemy.location]
        else:
            return ['move', rg.toward(robot.location, closestEnemy.location)]

        # move toward the center
        return ['move', rg.toward(robot.location, rg.CENTER_POINT)]

    def _flee(self):
        '''
        Bravely run away!
        If there is a nice chunk of empty space to move to close-by, run there.
        If not, run to the protection of a nearby friendly

        '''
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

    def _group_robots(self):
        # Extremely naive grouping algorithm that groups robots with their closest neighbour
        # starting with the closest two robots.  This can be sub-optimal if there are say 
        # 4 robots spread across the board with two of them close together at the center.
        # Ideally we'd pair each center one with one of the edge robots but that is much
        # more expensive to compute
        dists = dict()
        for robot1 in self.friendly_robots:
            for robot2 in self.friendly_robots:
                if robot1 == robot2:
                    continue
                if (robot2, robot1) in dists:
                    continue
                dists[(robot1, robot2)] = rg.wdist(robot1.location, robot2.location)

        groups = set()
        grouped = set()
        for i in range(int(math.ceil(len(self.friendly_robots) / 2.0))):
            smallest_distance = None
            smallest_bots = tuple()
            for bots, dist in dists.items():
                if bots[0] in grouped or bots[1] in grouped:
                    continue

                if smallest_distance == None:
                    smallest_distance = dist
                    smallest_bots = bots
                else:
                    #print `dist` + " " + `smallest_distance` + " id1: " + `bots[0].id` + " id2: " + `bots[1].id`
                    if dist < smallest_distance:
                        smallest_distance = dist
                        smallest_bots = bots

            if smallest_distance == None:
                # Odd number of robots created a loner. Group it with its closest ally
                loner = None
                for robot in self.friendly_robots:
                    if robot not in grouped:
                        loner = robot
                        break

                if loner == None:
                    print "Grouping the loner failed"

                nearest = self._closest_robot(self.friendly_id, loner.location)
                found = False
                for group in groups:
                    if nearest in group.robots:
                        group.robots.add(loner)
                        found = True
                        break;
                if not found:
                    print "Couldn't find loner's nearest group"
            else:
                #print "Grouped robots " + " id1: " + `smallest_bots[0].id` + " id2: " + `smallest_bots[1].id` + " with distance " + `smallest_distance`
                groups.add(RobotGroup((smallest_bots[0], smallest_bots[1])))
                grouped.add(smallest_bots[0])
                grouped.add(smallest_bots[1])
        return groups

# Not sure if we want to maintain a constant idea of all robots and their specific instances
class KnownRobot:
    next_id = 1
    def __init__(self, robot):
        self.id = KnownRobot.next_id
        KnownRobot.next_id += 1
        self.location = robot.get('location')
        self.hp = robot.get('hp')

class RobotGroup:
    def __init__(self, robots):
        self.robots = set(robots)

class Utils:
    @staticmethod
    def loc_to_key(loc):
        x, y = loc
        return `x` + ":" + `y`

    @staticmethod
    def loc_to_key2(loc):
        return (rg.settings.board_size * loc[1]) + loc[0]

    @staticmethod
    def new_robots_spawning(turn):
        return turn % 10 == 1