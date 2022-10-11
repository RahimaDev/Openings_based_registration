#include"Energy.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
//////////////////////////////////////////////////////////////
#include <boost/random.hpp>
#include <vector>
#include <random>
#include <numeric>
#include <functional>
#include <algorithm>
#include <iostream>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <queue>
#include <assert.h>
#include <fstream>
#include <time.h>       /* time */
using namespace std;
using namespace cv;
std::vector<DATA> read_data(std::string filepath)
{std::ifstream fichier(filepath, ios::in);
    std::ifstream fichier1(filepath, ios::in);
    
    std::vector<DATA> VEC;
    std::vector<Line>Lines;
    std::vector<pcl::ModelCoefficients::Ptr>plane;


    double A,B,C,D;
    int E,F, H,N,M;
    char S1,S2,S3,S4,S5,S6;

    if(fichier)
    {

        std::string line;
        while( getline(fichier, line))
        {
            std::istringstream buff(line);
            buff>>S1;
            if(char(S1)=='p')
            {
                buff>>S2>>A>>B>>C>>D>>E;
                pcl::ModelCoefficients::Ptr pln (new pcl::ModelCoefficients ());;

                pln->values.resize (4);
                pln->values[0]=(double)A;

                pln->values[1]=(double)B;
                pln->values[2]=(double)C;
                pln->values[3]=(double)D;
                plane.push_back(pln);
            }}}

    long double ax,ay,az, bx,by,bz;
    if(fichier1)
    {

        std::string line1;
        while( getline(fichier1, line1))
        {
            std::istringstream buff1(line1);
            buff1>>S2;
            if(char(S2)=='L')
            {Line l;
                buff1>>ax>>ay>>az>>bx>>by>>bz>>S5>>N>>M;
                //  std::cout<<ax<<std::endl;
                // std::cout<<(long double)ax<<std::endl;
                Eigen::Vector3d P1=Eigen::Vector3d ((double)ax,(double)ay,(double)az);
                Eigen::Vector3d P2=Eigen::Vector3d ((double)bx,(double)by,(double)bz);
                l.A=P1;
                l.B=P2;
                l.label=(char)S5;
                l.id_O=N;
                l.id_pl=M;

                Lines.push_back(l);}}}
    // std::cout<<"loulou"<<Lines.size()<<std::endl;
    for(int i=0; i<plane.size(); i++)
    {
        int index=i;
        DATA data;
        data.pl=plane[i];
        std::vector<Opening>OPP;
        std::vector<Line>LL;
        for(int m=0; m<Lines.size(); m++)
        {
            if(Lines[m].id_pl==index)
            {LL.push_back(Lines[m]);
                // std::cout<<LL[m].p1<<std::endl;
            }
        }
        do{
            int idx=LL[0].id_O;
            int s=0;
            Opening O;
            O.ln.push_back(LL[0]);
            O.id_pl=index;
            O.id_O=idx;
            LL.erase(LL.begin());
            for(int g=0; g<3; g++)
            {

                if(LL[0].id_O==idx)
                { O.ln.push_back(LL[0]);
                    LL.erase(LL.begin());}
            }

            OPP.push_back(O);
        }
        while(LL.size()>0);
        data.op=OPP;
        VEC.push_back(data);
    }

    
    return VEC;
}
//////////////////////////////////////////////////////////////////////////////////////////


void saveResultAsOBJ(std::string filename ,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // view_mutex_.lock();
    // view_reserve_mutex_.lock();



    // get filename
    //std::string filename = "new.obj";

    std::ofstream file;
    file.open(filename.c_str());

    size_t lineID = 0;
    size_t pointID = 1;
    std::map<size_t,size_t> lines2points;

    for(size_t i=0; i<cloud->points.size(); i++)
    {
        if((i%2)==0)
        {Eigen::Vector3d P1 =  Eigen::Vector3d(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
            Eigen::Vector3d P2 =  Eigen::Vector3d(cloud->points[i+1].x,cloud->points[i+1].y,cloud->points[i+1].z);

            file << "v " << P1.x() << " " << P1.y() << " " << P1.z() << std::endl;
            file << "v " << P2.x() << " " << P2.y() << " " << P2.z() << std::endl;

            lines2points[lineID] = pointID;
            ++lineID;
            pointID+=2;

        }
        else
            continue;
    }

    std::map<size_t,size_t>::const_iterator it = lines2points.begin();
    for(; it!=lines2points.end(); ++it)
    {
        file << "l " << it->second << " " << it->second+1 << std::endl;
    }

    file.close();

    //view_reserve_mutex_.unlock();
    //view_mutex_.unlock();
}
///////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr data_to_cloud(std::vector<Line>  C)
{ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<C.size(); i++)

    {
        pcl::PointXYZ pt1=pcl::PointXYZ(C[i].A.x(),C[i].A.y(),C[i].A.z());
        pcl::PointXYZ pt2=pcl::PointXYZ(C[i].B.x(),C[i].B.y(),C[i].B.z());

        cloud->points.push_back(pt1);
        cloud->points.push_back(pt2);}
    return cloud;

}

pcl::PointXYZ f_point( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ pt)
{
    double max=0;
    int j;
    for(int i=0; i<cloud->points.size(); i++)
    {
        double Val=pcl::geometry::distance(pt,cloud->points[i]);
        if(Val>max)
        {
            max=Val;
            j=i;
        }}
    pcl::PointXYZ p=cloud->points[j];
    return p;
}















int main (int argc, char** argv)
{
    cout << "Usage: " << argv[0] << " file1.ply file2.ply flie3.txt flie4.txt   d_thr " << endl;
    if(argc<1) return 1;

    int m = 1;
    string file1="", file2="", file3="", file4="";
    if (argc > m)
        file1 = argv[m++];

    if (argc > m)
        file2 = argv[m++];
    if (argc > m)
        file3 = argv[m++];
    if (argc > m)
        file4 = argv[m++];
    
    float d_thr;
    if (argc > m)
        d_thr = atof(argv[m++]);
    srand (time(NULL));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud6(new pcl::PointCloud<pcl::PointXYZ>);
    /////////////////////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud8(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud9(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud7(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud10(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud11(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud12(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(file1, *cloud1) == -1) //* load the file
    {

        return (-1);
    }
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(file2, *cloud2) == -1) //* load the file
    {

        return (-1);
    }
    

    std::vector<DATA> data1=read_data(file3);
    std::vector<DATA> data2=read_data(file4);


    std::vector<Line>ln1,ln2;
    std::vector<pair<pcl::ModelCoefficients::Ptr,Opening>> VEC1,VEC2;

    Eigen::Matrix4d Tr = Eigen::Matrix4d::Identity();

    Eigen::Matrix3d R2 = Eigen::Matrix3d::Identity();
    for(int i=0; i<data1.size(); i++)
    {
        DATA D=data1[i];
        std::vector<Opening>OP=D.op;
        for(int j=0; j<OP.size(); j++)
        {
            Opening O=OP[j];
            VEC1.push_back(make_pair(D.pl,O));
        }
    }


    for(int i=0; i<data2.size(); i++)
    {
        DATA D=data2[i];
        std::vector<Opening>OP=D.op;
        for(int j=0; j<OP.size(); j++)
        {
            Opening O=OP[j];
            VEC2.push_back(make_pair(D.pl,O));
        }
    }
    for(int i=0; i<VEC1.size(); i++)
    {
        std::vector<Line> L=VEC1[i].second.ln;
        for(int j=0; j<L.size(); j++)
        {

            ln1.push_back(L[j]);
        }}

    for(int i=0; i<VEC2.size(); i++)
    {
        std::vector<Line> L=VEC2[i].second.ln;
        for(int j=0; j<L.size(); j++)
        {

            ln2.push_back(L[j]);
        }}





    double Mini=100000000000;
    for(int g=0;g<80000; g++)
    {

        int a=rand() % VEC1.size();
        int b=rand() % VEC2.size();
        pair<pcl::ModelCoefficients::Ptr,Opening> D1=VEC1[a];
        pair<pcl::ModelCoefficients::Ptr,Opening> D2=VEC2[b];

        Opening O1=D1.second;
        Opening O2=D2.second;

        int e= rand()%2;
        int f= rand()%2;
        int n= rand()%2;
        int m= rand()%2;
        Line l1,l2,l3,l4;
        if(e==0)
            l1=O1.ln[0];
        else
            l1=O1.ln[3];
        if(f==0)
            l2=O1.ln[1];
        else
            l2=O1.ln[2];
        /////////////////////*
        if(n==0)
            l3=O2.ln[0];
        else
            l3=O2.ln[3];
        if(m==0)
            l4=O2.ln[1];
        else
            l4=O2.ln[2];

        Eigen::Vector3d u0=direct_vec(l1.A,l1.B);

        Eigen::Vector3d u2=direct_vec(l3.A,l3.B);

        Eigen::Vector3d v0=direct_vec(l2.A,l2.B);

        Eigen::Vector3d v2=direct_vec(l4.A,l4.B);
        std::vector<std::pair<Line, Line>>h;

        h.push_back(std::make_pair(l1,l3));
        h.push_back(std::make_pair(l2,l4));



        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();

        R0=Rotation(u0,u2,v0,v2);

        std::vector<Eigen::Vector3d> src,dst,dir;
        for(int m=0; m<h.size(); m++)
        {       Line L=h[m].first;
            Line L2=tr_line(L,R0);
            Eigen::Vector3d d=direct_vec(h[m].second.A,h[m].second.B);
            src.push_back(L2.A);
            dst.push_back(h[m].second.A);
            dir.push_back(d);

            src.push_back(L2.B);
            dst.push_back(h[m].second.A);
            dir.push_back(d);

        }



        T=pointToLine(src,dst,dir);

        double sum=Energy(ln1,ln2,T,R0,d_thr);
        if(sum<Mini)
        {

            Mini=sum;
            R2=R0;//Rot[j];
            Tr=T;

            std::cout<<Tr<<std::endl;
            std::cout<<Mini<<std::endl;

        }}

    std::cout<<Tr<<std::endl;
    std::cout<<Mini<<std::endl;



    Eigen::Isometry3d qt= Eigen::Isometry3d::Identity();
    Eigen::Vector3d t=Eigen::Vector3d(Tr(0,3),Tr(1,3),Tr(2,3));

    qt.linear()=R2;
    qt.translation()=t;

    Eigen::Vector4f centroid1;
    pcl::compute3DCentroid (*cloud1, centroid1);
    pcl::PointXYZ P1=pcl::PointXYZ(centroid1[0],centroid1[1],centroid1[2]);
    pcl::transformPointCloud (*cloud1, *cloud9, qt.matrix()
                              );
    for(int i=0; i<cloud1->points.size(); i++)
    {
        pcl::PointXYZ P;
        P.x=cloud1->points[i].x-P1.x;
        P.y=cloud1->points[i].y-P1.y;
        P.z=cloud1->points[i].z-P1.z;
        cloud5->points.push_back(P);
    }
    pcl::transformPointCloud (*cloud5, *cloud6, qt.matrix()
                              );

    for(int i=0; i<cloud6->points.size(); i++)
    {
        pcl::PointXYZ P;
        P.x=cloud6->points[i].x+P1.x;
        P.y=cloud6->points[i].y+P1.y;
        P.z=cloud6->points[i].z+P1.z;
        cloud7->points.push_back(P);

    }


    pcl::io::savePLYFile("TRRR.ply",*cloud7);




    return 0;
}






