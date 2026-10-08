# HRS Tutorial 2

This branch hosts the codebase of tutorial 2 of lecture "Humanoid Robot Systems".

## Build and Run

After building the devcontainer, **navigate to `hrs_ws` directory**, build and source the workspace as follows:

```bash
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

and then, make the python script executable:

```bash
chmod +x src/tutorial_2/scripts/tutorial_2.py
```

run the script with:

```bash
rosrun tutorial_2 tutorial_2.py
```

## Deliverable

For this assignment, the required figure is shown here.

![image](hrs_ws/src/tutorial_2/doc/Screenshot%20from%202024-10-24%2011-35-18.png)