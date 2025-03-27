import launch
import launch_ros.actions

def generate_launch_description():
  return launch.LaunchDescription([
    launch_ros.actions.Node(
      package="aeb",
      executable="safety_node_test",
      name="aeb_test_node",
      parameters=[{"test_throttle": 3.0 }],
      output="screen"
    ),
    launch_ros.actions.Node(
     package="aeb",
     executable="safety_node",
     name="safety_node",
     output="screen"
    )
  ])


# def generate_launch_description():
#   return launch.LaunchDescription([
#     launch_ros.actions.Node(
#       package="aeb",
#       executable="safety_node",
#       name="safety_node",
#       output="screen"
#     ),

#     launch_ros.actions.Node(
#       package="aeb",
#       executable="safety_node_test",
#       name="aeb_test_node",
#       parameters=[{"test_throttle": 2.5}],
#       output="screen"
#     )
#   ])
