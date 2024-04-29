from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    # Declare parameters
    sl.declare_arg('drone_num', '0')
    sl.declare_arg('x1', '10')
    sl.declare_arg('y1', '0')
    sl.declare_arg('x2', '0')
    sl.declare_arg('y2', '0')
    sl.declare_arg('x3', '-10')
    sl.declare_arg('y3', '0')
    sl.declare_arg('x4', '0')
    sl.declare_arg('y4', '-10')
    sl.declare_arg('z', '15')
    sl.declare_arg('yaw', '0')
    sl.declare_arg('pitch', '0')
    sl.declare_arg('roll', '0')
    sl.declare_arg('frame_id', 'usv/base_link')
    sl.declare_arg('child_frame1', 'uav1/target')
    sl.declare_arg('child_frame2', 'uav2/target')
    sl.declare_arg('child_frame3', 'uav3/target')
    sl.declare_arg('child_frame4', 'uav4/target')

    sl.include(package = 'ecn_2023', launch_file = 'world_launch.py')

    sl.include(package = 'ecn_usv', launch_file = 'usv_launch.py')

    for i in range(1, 5):
        # Loop to create nodes for UAVs and static transforms
        sl.node(package='ecn_2023', executable='uav', parameters={'drone_num': i})

        sl.node(package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[sl.arg('x' + str(i)), sl.arg('y' + str(i)), sl.arg('z'), sl.arg('yaw'), sl.arg('pitch'), sl.arg('roll'), sl.arg('frame_id'), sl.arg('child_frame'+ str(i))])

    return sl.launch_description()
