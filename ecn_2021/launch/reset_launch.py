from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('rsp', default_value=False)
    
    poses = ((2.5,0.,0.),(2.,0.,1.),(3.,1.,-1.))
               
    for i,(x,y,t) in enumerate(poses):
        
        turtle = f'turtlebot{i+1}'
        
        r = 255 if i == 0 else 0
        g = 255 if i == 1 else 0
        b = 255-r-g     
        laser = [r,g,b]
        
        with sl.group(ns=turtle):
            with sl.group(if_arg='rsp'):
                sl.robot_state_publisher('ecn_2021', 'turtlebot3_waffle_pi.urdf.xacro', xacro_args={'prefix': turtle})
            
            sl.node('map_simulator', 'spawn', parameters = {'zero_joints': True, 'static_tf_odom': True, 'radius': .15, 'x': x, 'y': y, 'theta': t, 'robot_color': [50+v//5 for v in laser], 'laser_color': laser})
    
    return sl.launch_description()
