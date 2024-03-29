def parse_ugv_goals(filename):
    goals = []
    offset=3.0
    with open(filename, 'r') as file:
        lines = file.readlines()[1:]  # Skip the header line
        for line in lines:
            line = line.strip()
            if line:
                parts = line.split(',')
                x = float(parts[1])
                y = float(parts[2])
                z = 0    #float(parts[2])
                goal_type = parts[3].strip()
                uav_id = int(parts[4].strip()) if len(parts) > 4 else None
                goals.append((x, y+offset, goal_type, uav_id))
                #goals.append((x, y))
    return goals

def get_ugv_goals():
    ugv_goals_file = '/home/experiment/catkin_ws/src/arl/src/experiments/ugv_goals_July_24_2023_f3_avg.txt'
    goals = parse_ugv_goals(ugv_goals_file)
    return goals
