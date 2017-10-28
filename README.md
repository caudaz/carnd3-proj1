# **Self-Driving Car Engineer Nanodegree**

# **Term3 – Project1: Path Planning** #

![C:\\Users\\ab\\AppData\\Local\\Microsoft\\Windows\\INetCache\\Content.Word\\2017-10-28
08\_57\_50-Microsoft PowerPoint -
\[carnd3-proj1.pptx\].png](./media/image1.png)


# **INTRODUCTION** #

The purpose of path planning is creating smooth and safe trajectories.
For this project the car had to be able to travel around 50mph around
the track while passing cars safely.

![C:\\Users\\ab\\AppData\\Local\\Microsoft\\Windows\\INetCache\\Content.Word\\2017-10-28
08\_21\_00-highway\_map\_plot.png - Picasa Photo
Viewer.png](./media/image2.png)


# **PATH PLANNING IMPLEMENTATION** #

For this implementation, the path planning model shown in class was
followed:

![C:\\Users\\ab\\AppData\\Local\\Microsoft\\Windows\\INetCache\\Content.Word\\2017-10-28
08\_59\_54-Lesson 04 - Behavior Planning - Google
Drive.png](./media/image3.png)

-   Behavior was implemented using states.

-   Prediction used sensor fusion data + localization (for the SDC and
    other vehicles on the road) for current state prediction and
    checking for safety.

-   Trajectory generation used splines.

# **BEHAVIOR** #

Behavior is implemented by using a finite set of states:

![C:\\Users\\ab\\AppData\\Local\\Microsoft\\Windows\\INetCache\\Content.Word\\2017-10-28
09\_01\_55-Microsoft PowerPoint -
\[carnd3-proj1.pptx\].png](./media/image4.png)

-   Cruise – no car in front, set speed to ~50MPH

-   Change lane – car in front, set speed depending distance to car

-   Do not block left lane – on left lane for more than 3 secs, move to
    right lane

# **PREDICTIONS** #

Predictions were used:

-   Going into the behavior model: predicting the speed and location of
    the car in front

-   Into the trajectory generation model: predicting cars on other lanes
    left/right speed and position, to make sure that a safe lane switch
    was possible.

# ****TRAJECTORY GENERATION**** #

![C:\\Users\\ab\\AppData\\Local\\Microsoft\\Windows\\INetCache\\Content.Word\\2017-10-28
09\_11\_04-self\_driving\_car\_nanodegree\_program.png](./media/image5.png)

The trajectory generation was implemented using a spline. The spline
used:

-   For the 2 initial points: the last 2 points of the unused path

-   For the last 3 points: 3 points spaced apart 27m, found using the
    getXY function that takes the S position + lane combined with the
    map of the track (LOCALIZATION).

The spline path ensures continuous derivatives for speed, acceleration,
and jerk. The 27m spacing apart seem to be the smallest distance that
would ensure that latAcc would stay within the 10m/s2 limits.

# **CONCLUSIONS** #

A very simple model using a combination of a set of finite states and
spline generation could get the car drive around the track at the
desired sped without incidents.

# **POTENTIAL ENHANCEMENTS TO THE MODEL** #

-   Instead of using a simple set of states, use a more comprehensive
    set such adding: prepare for switch lanes left and prepare for lanes
    right.

-   Instead of using simple speed and location predictions, use S, d and
    time components (for the SDC and other vehicles) to check for
    potential collisions using S and d plane intersections.

-   Instead of using a fixed spline (constant curvature), use Jerk
    Minimizing Trajectory (JMT) to ensure the smoothest lane switching
    that would vary according to the situation.

-   Implement feasibility of the maneuver optimization, checking:
    long/lat. acceleration, speed, and curvature of the road (as a
    function of the steer angle).


