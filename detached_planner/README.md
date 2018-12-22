# multi agent simulation
WIP
ROS package, that shows complete detached planner out of ROS nav_core package
All copyright goes to makers of nav_core
In this pakage i did not implement anything, just showed how to detach it

## Getting Started

Installing: git clone git@bitbucket.org:AlexKaravaev/multiple_agent_planner.git

### Prerequisites

What things you need to install the software and how to install them

```
ros deps:
  nav_core
  nav_core_msgs
  fake_localization

```


## Usage
Set this parameter
```
<param name="base_global_planner" value="detached_planner/GlobalPlanner" />
```
