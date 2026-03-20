# TODOs for the controller and simulator

1. Fill in the rest of the controller logic
    1. Rotate on startup (how coordinate with nav stub? Could send cmd_vel direct)
    2. Explore action 
    3. Handle action-in-flight checks - will get quite flabby...
    ~4. Manipulator~ 
    ~5. Return to bin - should be easy~
    ~6. Continue~
2. Manipulator stub
    ~1. Just "acquire" block if in range~
    2. Can just handle in controller at first - but move to proper action server to mock the action integration
3. Integrate LIDAR mocking for the navigation stub
    1. Mike sorting
4. Integrate Depth camera mocking for the CV node proper 
5. Integration with actual ROS packages as they come in. They must work in sim.
6. Proper frames and tf transforms
7. Remaining action definitions (navigation and manipulator)
8. Visualisation setup
    1. Current map and robot location in map 
    2. Current estimates of block positions (and status)
    3. Current estimates of bin positions 
    4. Current phase of controller 
~9. Cut back on some of the spammy logging out of the controller nodes - maybe set a heartbeat message which each node can set? Or something~
10. The controller ignores block updates during some moves, but that doesn't stop the smoothing node from updating them - which the controller will then ingest the next time it moves to a new state. May or may not be a problem (in principle the block will be deposited by the time the controller accepts new observations)
11. Add obstacles to the map


# TODOs in the code 

- What if the navigation fails? Execute one explore step and try again?
- Tracking collection - the observation smoother doesn't know if something is collected or not. So remove that field, and track it separately in the controller
- Need to consider robustness to state changes from different positions in the FSM. What if the manipulator sets us back to explore?
- When we run the longer-term policy, the rover ends up in the wrong location. Is that because, even in sim, /odom drifts? Or what else could be going on? How could I visualise it? What are the positions relative to? Would this be a problem if I'm maintaining things in the map frame?

# NEXT STEPS
NEXT: sim? something idk? Maybe more stuff on frames and tf transforms?

- Q - is it time to start working on the visualisation setup in rviz?
- IDEA - if the smoothing node finds a new node far from an old one with the same colour as another one, overwrite it - may be able handle the SLAM drift
