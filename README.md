#Openings based registration framework by Rahima Djahel(ENPC), Bruno Vallet(IGN) and Pascal Monasse(ENPC)
#Required dependencies: PCL, Eigen, CGAL,Boost

O_Registration: an efficient algorithm to register an indoor and outdoor scan.

Test:
./O_Registration ../data/indoor_scan.ply ../data/STR721_Int.ply ../data/int_lines.txt ../data/out_lines.txt 5

where:

indoor_scan.ply: the indoor scan.

STR721_Int.ply: indoor points detected from exterior scans.

int_lines.txt: file contains information about detected openings from the indoor scan.

out_lines.txt: file contains information about detected openings from the outdoor scan.

5 : distance threshold (can be adapted by the users).


#Remark :


As the size of the 3D point cloud representing the indoor points seen from the outdoor is very small compared to the size of the indoor scan, we preferred to use two different thresholds of inliers (to detect planar polygons) as well as two different values for the minimum size of a planar region.

that's why we have fixed these two parameters in Poly_registration.cpp in order to be compatible with our data size.

#to visualize the result:

cloudcompare.CloudCompare TRRR.ply ../data/STR721.ply

#If you use our algorithm in any of your publications or projects, please cite our paper:

DJAHEL, Rahima, VALLET, Bruno, et MONASSE, Pascal. DETECTING OPENINGS FOR INDOOR/OUTDOOR REGISTRATION. The International Archives of Photogrammetry, Remote Sensing and Spatial Information Sciences, 2022, vol. 43, p. 177-184.

#If you have any questions, you can send an email to :

rahima.djahel@enpc.fr

rdjahel@gmail.com
