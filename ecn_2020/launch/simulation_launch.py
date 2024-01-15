from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    
    # load descriptions
    for robot in ('r2d2', 'bb8','d0'):
        with sl.group(ns = robot):
            sl.robot_state_publisher('ecn_2020', robot + '.xacro')
    
    # run Python simulation
    sl.node('ecn_2020','simulation.py', output='screen')
    
    # run RViz2
    sl.rviz(sl.find('ecn_2020', 'config.rviz'))
    
    return sl.launch_description()
