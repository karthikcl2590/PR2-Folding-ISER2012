olding
This is where all actual folding motions are done; in the basic package, it assumes the cloth polygons are specified by human input. In addition to the typical srvs/msgs packages, there are 3 main pieces here.

folding_gui <will be changed to folding_geometry tomorrow>: Since our approach to folding is almost entirely geometric, all the work done in defining the folds / computing fold lines is done via 2D geometry. This package is where that happens. FoldingGUI.py provides a graphical front end to polygon specification and folding. poly_gui_bridge.py interfaces between 2D and 3D representations: it takes in 3D points and fits a 2D polygon to them for the FoldingGUI to work with, then converts fold specifications from the GUI into 3D points for the robot.

Note that poly_gui_bridge.py takes a stream of points as input. It knows how many vertices it expects from a polygon, and will keep reading 3D points ("click_points" I think, see the stereo_click package) til it's hit the right number. Then it makes a polygon and sends it to the folding app. This was done so it'd be easy to swap out human-in-the-loop (via stereo_click, sending points one at a time) or automated detection (where our vision code will send it the vertices of the polygon in clockwise order).

folding_execution: Once we've computed the fold lines and necessary grasp points to execute for a given polygon, we need the robot to be able to follow a 3D trajectory. These motions are defined by fold_executor.py. (You can ignore the other python files. They've been phased out.)

folding_apps: This is what combines the two together. As I recall it just houses launch files.

Sanity Check: Try running "roslaunch pr2_simple_motions_apps gui_interface.launch". That will launch a FoldingGUI, as well as fold_executor.py and a poly_gui_bridge to plug them together. It will also launch a stereo view of the cameras. If you click on N corresponding points in both images (where N=4 if the launch file is set to towel mode, 7 if set to pants, and 10 for shirts and sweaters), you should see a polygon pop up in the GUI. You can drag fold lines on it and watch it send the PR2 motions accordingly.

visual_feedback.
This contains all of the vision used in the folding procedure, and it's also the one which is most desperately in need of better documentation (and removing some deprecated packages). The crux:

image_processor: Provides a generic framework for realtime image processing on the robot, both monocular (process_mono.py), stereo  (process_stereo.py), and via optical flow (process_optical.py, still testing).

visual_feedback_utils: Contains python files for background segmentation, contour tracing, and shape-model fitting. By default we assume the object in question is on a green background, though it also can handle white or black if set properly.

passive_shape / cloth_models (in transit but I think it's split among these): Where code from our ICRA paper on Parametrized Shape Models for Clothing comes from. This is how it detects the cloth polygon.
