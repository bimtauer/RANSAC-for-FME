This is my ongoing project of building a Python RANSAC (almost) from scratch since I grew impatient with all the workarounds I had to come up with to make python-pcl work for me. My approach so far has been tailored towards using this RANSAC directly within FME to automate shape detection tasks in point clouds, for example inventories of urban infrastructure such as light- and signposts. 
However the RANSAC class and its corresponding geometry models are to be designed as a lone standing tools that can be flexibly used.
In the future I'd like to clean up the existing code, potentially add more shape classes one can search for and most of all further improve speed. 
Implementing an own octree-based normal vector estimation would also be great. 
If anyone feels like helping with this, have a go!
