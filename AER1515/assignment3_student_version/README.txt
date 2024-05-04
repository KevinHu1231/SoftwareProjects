No Additional Dependencies from the starter code. First run part 1, then part 2 on all 5 test images, then part 3. Part 1 will generate depth maps in the test est_depth folder. 
Part 2 will generate bounding boxes and labels in the test bbox folder and the bblabel folder respectively for a single image. 
Make sure the image path is correct for the image you want to generate the bounding box for. 
Part 3 will generate segmentation masks for the test images based on the depth images from Part 1. 
Programs can be changed to training mode by using Train = True in Part 1 and Part 3 and using train instead of test for the paths in Part 2.
All depth maps, bounding box images, labels and segmentation masks have already been generated and in their designated folder.