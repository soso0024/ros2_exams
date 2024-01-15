from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    for robot, target in [('bb8','r2d2'), ('d0','bb8')]:
        with sl.group(ns = robot):
            sl.node('ecn_2020', 'control',
                parameters = {'frame': robot+'/base_link', 'target': target+'/base_link'})
    
    return sl.launch_description()
