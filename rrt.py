import math

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics
from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, compute_surface_aabb, get_sample_fn, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, \


from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

GOAL_SAMPLE = 2
POS_STEP_SIZE = 0.01

class Tree:
    def __init__(self, point):
        self.parent = None
        self.children = []
        self.point = point
    
    def add_child(self, child):
        self.children.append(child)
        child.parent = self
    
    def set_parent(self, parent):
        parent.add_child(self)
        
    @property
    def size(self):
        if len(self.children) == 0: return 1
        return sum([child.size for child in self.children]) + 1

class TrajectoryGenerator:

    def __init__(self, world):
        self.world = world
        self.tool_link = link_from_name(world.robot, 'panda_hand')
        self.sample_free = get_sample_fn(world.robot, world.arm_joints)

    def rrt(self, goal_pose):
        start_pose = get_link_pose(self.world.robot, self.tool_link)
        tree = Tree(start_pose)
        count = 1
        while True:
            rand_point = self.sample_free(end_region) if count % GOAL_SAMPLE == 0 else sample_free(bounds) # goal biasing
            nearest = find_nearest(tree, rand_point)
            new_point = steer(nearest.point, rand_point)
            if obstacle_free(environment, radius, nearest.point, new_point):
                last_node = Tree(new_point)
                nearest.add_child(last_node)
                if end_region.contains(Point(new_point)): break
            count += 1
        final_path = find_path(last_node)

        return final_path

    def find_nearest(tree, point):
        if len(tree.children) == 0: return tree
        return min([tree] + [find_nearest(child, point) for child in tree.children], key=lambda x: math.dist(point, x.point))
        
    def steer(self, nearest_point, rand_point):
        distance = math.dist(nearest_point, rand_point)
        if distance <= DIST_LIMIT:
            return rand_point
        new_x = (rand_point[0] - nearest_point[0]) * (DIST_LIMIT / distance) + nearest_point[0]
        new_y = (rand_point[1] - nearest_point[1]) * (DIST_LIMIT / distance) + nearest_point[1]
        return new_x, new_y

    def obstacle_free(environment, radius, nearest_point, new_point):
        line = LineString((nearest_point, new_point)).buffer(radius, resolution=3)
        for obs in environment.obstacles:
            if line.intersects(obs): return False
        return True

    def find_path(self, goal):
        curr = goal
        path = [curr.point]
        while curr.parent:
            curr = curr.parent
            path.append(curr.point)
        path.reverse()
        return path

    def path_length(self, path):
        length = 0
        for i in range(len(path) - 1):
            length += math.dist(path[i], path[i + 1])
        return length
