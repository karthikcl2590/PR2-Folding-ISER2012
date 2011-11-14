# Steps to Run Stephen's Towel Folding Demo

Code is in the `unfolding_smach` package.

## Prepwork

- Make sure runstop and controller charged
- Smooth the table so that fabric is taut, tucking loose sides under table

## PR2 Placement

- Raise torso all the way up
- Drive as close as possible to table with "chest" still being able to move
  in and out in the y direction. Make sure perfectly parallel to table.
- On rare occasion, may have to redo cropping of table
  - `camview /wide_stereo/left`
  - Click to get `frame0000.jpg` in directory
  - Open in Gimp
  - Determine good rectangle of table to crop (`top left x, y`, `width`, `height`)
  - Modify values at top of vision.launch accordingly

## Launching

- Launch on c1/c2 if possible for faster processing

        roslaunch pr2_simple_motions_apps move_all.launch
        roslaunch unfolding_smach table.launch 
        roslaunch unfolding_smach stances.launch
        roslaunch unfolding_smach vision.launch

- OR

        roslaunch unfolding_smach full_demo.launch

## Execution

- Finally, to execute, use `rosrun unfolding_smach demo.py`
- Within the `GrabTriangle` state you can see that it will prompt you for
  whether or not it got the front/back triangles right, just explain purpose
  when giving demo

## Debugging

- Much easier to debug by specifying start state for state machine in `demo.py`
- Parameters to debug
  - Table height
  - Values in `SmachUtils.py` and `GripUtils.py`, various offsets for calibration
    errors, can also speed up motions (SPEEDUP variable in GripUtils.py)
  - Offsets for grabbing corners in `PickupCorner` in `demo.py`

## Improvements/TODOs

- Pickup/laydown prior to triangle fitting
- Detecting table plane/height, perhaps with Kinect/blob detection of green
- Can probably replace shake
- Reliability of corner grabbing (approach angle)
- Easier way of adjusting speed (reliability trade-off) depending on demo time constraints
- Twisting wrist as in Jeremy's original implementation
