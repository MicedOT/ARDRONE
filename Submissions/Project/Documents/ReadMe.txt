Documentation
- Report
- ReadMe
  Contains a guide and documentation of different folders in the folder 

PreProcessing
  - bag_to_csv 
  - bag_to_images
  - combine

Phase 1
  Image Detection
    - center_orientation
      returns the center and the orientation of the key locations in the images in pixels

  Obstacle Detection
    - center_radius
      returns the center and the radius of the obstacles in the images in pixels

  Transformation
    - pixel_to_world
      converts pixel coordinates to world coordinates hence returning global center and radius  
    - pixel_to_world_images
      converts pixel coordinates to world coordinates hence returning global center and global orientation 
 
  Outputs
    centroid
      - centroids
      - image_centroids
    Detections
      - center_radius
      - pic_orientation
    Global Correspondance
      - global_coordinates
        Global Centers and Radius of Obstacles
      - global_image_coordinates
        Global Centers and Orientations of key Images

  Miscellaneous
    Kmeans codes,  Display codes along with combining files codes

  Metadata
    - Data required for image processing 

Phase 2
  Gazebo
    - Base modified Gazebo World file
  Launch
    - Base Modified Launch Files
  Scripts
    - Base Modified Scripts
    - rrt
      rrt path planning with smoothening
    - order
      Edit order to change order in which the drone flies to images


Execution Guidelines

*Replace line numbers 66 in desired positions and 75 in rosinterface to yield your own paths to order.txt

In order to run the scripts change the order to your liking with each picture on a new line and click: 
X -> Downward Camera
O -> to get ready to send it to [1,1,2] 
U -> To  Start Processing
Once it is steady -
B -> To Execute Project Path

