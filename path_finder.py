import math

def beauty_print(arr):
    for row in arr:
        print(*row, sep="\t")

# implementing path finding algorithm for the robot
num_obstacles = 13
obstacles = [
(0.61, 2.743),(0.915, 2.743),(1.219, 2.743),(1.829, 1.219),
(1.829, 1.524),( 1.829, 1.829), (1.829, 2.134),(2.743, 0.305),
(2.743, 0.61),(2.743, 0.915),(2.743, 2.743),(3.048, 2.743),
(3.353, 2.743),
(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),(-1,-1),
(-1,-1),(-1,-1),(-1,-1),
]
start = (0.305, 1.219)
goal = (3.658, 1.829)

block_length = 0.305
max_block_len_ind = 16
max_block_wid_ind = 10
total_blocks = max_block_len_ind * max_block_wid_ind
init_potential = total_blocks + 5
obstacle_potential = 500
config_space = [[init_potential for i in range(max_block_len_ind)] for j in range(max_block_wid_ind)]
for i in range(num_obstacles):
    len_obs, wid_obs = obstacles[i]
    len_block, wid_block = round(len_obs/block_length), round(wid_obs/block_length)
    for w in range(2):
        for l in range(2):
            new_w = max_block_wid_ind - (wid_block - w) - 1
            new_l = len_block - l
            if new_l >= 0 and new_l < max_block_len_ind and new_w >= 0 and new_w < max_block_wid_ind:
                config_space[new_w][new_l] = obstacle_potential

start_block = round(start[0]/block_length), round(start[1]/block_length)
goal_block = round(goal[0]/block_length), round(goal[1]/block_length)


def path_finder(obstacles, start, goal):
    goal_obj = (max_block_wid_ind - goal[1] - 2, goal[0] - 1, 0) # (width_index, length_index, 0)
    q = []
    q.append(goal_obj)
    visited_nodes = set()
    while q:
        obj = q.pop(0)
        config_space[obj[0]][obj[1]] = obj[2]
        visited_nodes.add((obj[0], obj[1]))
        for w in range(-1, 2, 1):
            for l in range(-1, 2, 1):
                new_w = obj[0] + w
                new_l = obj[1] + l
                new_p = obj[2] + 1
                if abs(w) == abs(l):
                    continue
                if new_w >= 0 and new_l >= 0 and new_w < max_block_wid_ind and new_l < max_block_len_ind:
                    if (new_w, new_l) not in visited_nodes and config_space[new_w][new_l] != obstacle_potential and config_space[new_w][new_l] > new_p:
                        q.append((new_w, new_l, new_p))
    return 

path_finder(obstacles, start_block, goal_block)

beauty_print(config_space)