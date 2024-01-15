from simple_launch import SimpleLauncher
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    
    sl = SimpleLauncher()
        
    map_file = sl.find('ecn_2021', 'house.yaml')
    
    robot = 'turtlebot1'
    
    sl.node('nav2_map_server','map_server',name='map_server',
        parameters={'yaml_filename': map_file})
    sl.node('nav2_lifecycle_manager','lifecycle_manager',name='map_server_manager',
    output='screen',
    parameters={'autostart': True, 'node_names': ['map_server']})
        
    def configure(rewrites):
        return
    
    with sl.group(ns=robot):
        nav_nodes = [
                ('nav2_controller','controller_server'),
                ('nav2_planner','planner_server'),
                ('nav2_recoveries','recoveries_server'),
                ('nav2_bt_navigator','bt_navigator'),
                ]
        
        configured_params = RewrittenYaml(
                            source_file=sl.find('ecn_2021', 'turtlebot1_nav_param.yaml'),
                            root_key=robot,
                            param_rewrites={'default_bt_xml_filename': sl.find('nav2_bt_navigator','navigate_w_replanning_time.xml')},
                            convert_types=True)
        
        lifecycle_nodes = []
        for pkg,executable in nav_nodes:
            
            sl.node(pkg, executable,name=executable,
                parameters=[configured_params])
            
            lifecycle_nodes.append(executable)

        sl.node('nav2_lifecycle_manager','lifecycle_manager',name='nav_manager',
        output='screen',
        parameters=[{'autostart': True,
                    'node_names': lifecycle_nodes}])

    return sl.launch_description()
