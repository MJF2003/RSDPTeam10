# TODOs for the controller and simulator

~1. Fill in the rest of the controller logic~
    ~1. Rotate on startup (how to coordinate with nav stub? Could send cmd_vel direct. Now think this will be simplest - although will require a new action server.)~
    ~2. Explore action~ (Haotian sorting)
    ~3. Handle action-in-flight checks - will get quite flabby...~
    ~4. Manipulator~ 
    ~5. Return to bin - should be easy~
    ~6. Continue~
~2. Manipulator stub~
    ~1. Just "acquire" block if in range~
    ~2. Can just handle in controller at first - but move to proper action server to mock the action integration~
~3. Integrate LIDAR mocking for the navigation stub~
    ~1. Mike sorting~
~4. Integrate Depth camera mocking for the CV node proper~ 
5. Integration with actual ROS packages as they come in. They must work in sim.
~6. Proper frames and tf transforms~
~7. Remaining action definitions (navigation and manipulator)~
8. ~Visualisation setup~
    1. ~Current map and robot location in map~ 
    2. ~Current estimates of block positions (and status)~
    3. ~Current estimates of bin positions~ 
    4. ~Current phase of controller~ 
~9. Cut back on some of the spammy logging out of the controller nodes - maybe set a heartbeat message which each node can set? Or something~
10. The controller ignores block updates during some moves, but that doesn't stop the smoothing node from updating them - which the controller will then ingest the next time it moves to a new state. May or may not be a problem (in principle the block will be deposited by the time the controller accepts new observations)
~11. Add obstacles to the map~ (Mike handling)
~12. Update sim to use Jinwei message definitions~
~13. Test new rover model in sim.~
14. In sim, the vision node only recognises blocks, not bins
  For now use this 
```bash
ros2 topic pub /cv/bin_poses rover_interface/msg/BinPoseObservation "{
  header: {frame_id: 'map'}, 
  yolo_fps: 30.0, 
  observations: [
    {
      id: 1, 
      pose: {position: {x: -1.2, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, 
      color: 3
    }, 
    {
      id: 2, 
      pose: {position: {x: -1.2, y: 0.55, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, 
      color: 2
    },
    {
      id: 3, 
      pose: {position: {x: -1.2, y: -0.55, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, 
      color: 1
    }
  ]
}" -1
```

# TODOs in the code 

- What if the navigation fails? Execute one explore step and try again?
- What about manipulation?
- Tracking collection - the observation smoother doesn't know if something is collected or not. So remove that field, and track it separately in the controller
- Need to consider robustness to state changes from different positions in the FSM. What if the manipulator sets us back to explore?
- When we run the longer-term policy, the rover ends up in the wrong location. Is that because, even in sim, /odom drifts? Or what else could be going on? How could I visualise it? What are the positions relative to? Would this be a problem if I'm maintaining things in the map frame?
- The vision node (in sim) seems to be identifying blocks with unknown colours. How do we want to handle that case? And are they real blocks?

DONE: ~fix explore server waiting for map~ 
DONE: ~alternate map which places a wall between the rover and the blocks~ 
DONE: ~Optional argument to the rover_description, so that in sim we can just not render the arm (because it's making everything fucking slow)~
DONE: ~Can we ignore some stuff in the colcon build?~
DONE: ~For the sim vibe test - we want to cancel exploration when we see a block.~
DONE: ~Controller needs to update the costmap with block/bin positions~



# NEXT STEPS
- IDEA - if the smoothing node finds a new node far from an old one with the same colour as another one, overwrite it - may be able handle the SLAM drifts

TODO: If the controller sees a bin observation on top of a block observation, then it should probably assume that that is a bin and overwrite the block observation?
TODO: Controller code - if the vision recognitions are moving then it can run into the propose_nav_pose - I think that's just because it doesn't know the bin though.
TODO: Chunyi code:
  - Convert to action server 
  - Test with smoothed nodes
