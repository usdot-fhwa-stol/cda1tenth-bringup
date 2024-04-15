# Navigation 2

[Navigation 2 (Nav2)][nav2] is a motion planning, behavior planning, and general autonomy framework for ROS 2. It
builds on top of the ROS 2 Action interface and uses the BehaviorTree.CPP library for its decision making. Nav2 is
complex with several moving parts. The content below aims to provide readers with a high-level overview, supplementing
the official documentation.

[nav2]: https://navigation.ros.org/

## General flow

The following describes a basic overview of how navigators (and, more generally, Nav2) operate. Depending on the
application, this process can be significantly more complex.

1. Generate a path from the robot's current pose to the specified goal pose.
   - The navigator sends an action request to the active planner server plugin via a ROS 2 action and stores
     the returned result.
   - The planner server independently maintains an understanding of the environment through global costmap.
2. Track the reference path.
   - The navigator sends an action request to the active controller server plugin via a ROS 2 action.
   - The controller server independently maintains an limited understanding (within some distance to the robot) of the
     environment through a local costmap.
3. React to operating changes.
   - The navigator can request a new path or cancel a control effort, among other preemptive actions.
   - The planner or controller server plugins can report back to the navigator when actions fail, and the
     navigator can take appropriate corrective steps.

## Behavior trees

A [behavior tree][behavior_tree_wiki] (BT) is a mathematical model that describes how a robot plans its actions. The
tree's leaf nodes represent different actions, and inner nodes control which of these actions the robot will execute.

[BehaviorTree.CPP][behavior_tree_cpp] is a C++ library for working with behavior trees. The library is essentially a
lightweight task orchestrator; it decides which actions to take, but the actual computation gets dispatched to
separate threads.

While the BehaviorTree.CPP library is independent from ROS, Nav2 extends it with custom BT Action nodes (and non-Action
nodes). The custom BT Action nodes provide a ROS-compatible interface, allowing the library to dispatch work via ROS 2
actions.

[behavior_tree_wiki]: https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control)
[behavior_tree_cpp]: https://www.behaviortree.dev/

## Behavior Tree (BT) Navigator Server

This server contains _navigators_: decision makers that manage calls to the other servers to provide high-level
autonomy. Nav2 comes with two navigators: `NavigateToPose` and `NavigateThroughPoses`. Both navigators load behavior
trees from XML files, and users can customize each navigator's functionality through these XML files. Additionally,
users can provide their own navigators if they desire.

For an example, consider the following behavior tree specification:

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
      <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector"/>
      <DistanceController distance="1.0">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}"/>
      </DistanceController>
      <FollowPath path="{path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}"/>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

From this specification, the `NavigateToPose` navigator will behave as follows:

1. Make a service call to the Controller Server to activate the specified controller plugin. (See `params.yaml` for
   more information.)
2. Make a service call to the Planner Server to activate to specified planner plugin. (See `params.yaml` for more
   information.)
3. Make an action call to the Planner Server to generate a new reference path to track. The navigator will request a
   new reference path whenever the robot is greater than 1 meter from the goal pose.
4. Make an action call to the Controller Server to track the reference path.

The navigator will repeat the pipeline until the robot reaches its gaol pose.

> [!NOTE]
> The Planning and Control Servers' plugins do the actual planning and tracking, respectively. The navigator, which
> lives within the BT Navigator Server, only decides when to send the corresponding ROS 2 action requests.

## Planning Server

This server contains a collection of path planner plugins that generate a path from the robot's current pose to some
goal pose. The plugins expose a ROS 2 action interface for generating a path through free-space. Users can choose
one of the planners that comes with Nav2, or they can bring their own.

The server maintains an understanding of the robot's environment through a global costmap.

## Controller Server

This server contains a collection of controller plugins that command the robot to follow the provided reference path.
Each plugin exposes a ROS 2 action. Similar to the Planning Server, users can choose from several provided controller
plugins, or they can implement their own.

They are responsible for publishing `geometry_msgs::msg::Twist` messages onto the `/cmd_vel` topic.

### Dealing with different robot types

The Controller Server's plugins publish `geometry_msgs::msg::Twist` messages. This works fine for holonomic robots,
but the message type presents issues for Ackermann or similarly-constrained platforms. There are two options for
dealing with this challenge:

1. convert the `Twist` messages to `ackermann_msgs::msg::AckermannDrive` messages (or a similar message type) using an
   adapter node
2. abuse the `Twist` message's fields to contain the desired information

Option 1. is preferable because it better documents the interface changes. Check out
[_Is "Twist" (still) a good velocity command interface_][discourse_link] for lengthy discussion on the two
possibilities.

[discourse_link]: https://discourse.ros.org/t/is-twist-still-a-good-velocity-command-interface/13218

## Behavior Server

This server contains a collection of various behaviors. Navigators typically use they behaviors as recovery moves when
path planning and tracking fails. Nav2 provides some behaviors, including waiting, backing up, or calling for help.
Unlike the Planning and Controller Servers, the Behavior Server's plugins do not have a unified interface.

## Route Server

This is an experimental server that plans routes using well-structured graphs, such as road graphs, instead of
free-space. Users provide a graph file that gets overlaid on top of the static map file.

In an ideal environment, the navigator could completely bypass the Planner Server, calling the Router Server to
generate a route through the graph and an associated dense path then feeding that to the Controller Server plugin to
track. However, there are some scenarios that would require Nav2's free-space planning capabilities:

- If the distance between the robot and the route's beginning is significant. This is an example of the last-mile
  problem.
- If the route has a blockage and the robot needs to maneuver around it.

The navigator's general data/processing flow when using the Route Server is:

1. Make an action call to the Route Server to generate a route and associated dense path through the loaded graph.
2. (Optional) If the route's start is too far away from the robot's current pose, make an action call to the Planner  
   Server to generate a path in free-space between the robot's current pose and the route's start.
3. (Only if 2.) Make an action call to the Controller Server to track the reference path generated in 2.
4. Make an action call to the Controller Server to track the reference path generated in 1.

> [NOTE!]
> The `nav2_route` package has not been merged into the main Nav2 code base. It's integration with the rest of Nav2 is
> limited to a few demonstration scripts that use the `nav2_simple_commander` package. This package provides a Python
> API for creating navigators outside the BT Navigator Server.
