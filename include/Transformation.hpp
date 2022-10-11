

#include"clustering.hpp"








Eigen::Vector3d V_proj(Eigen::Vector3d U1, Eigen::Vector3d V1)
        { 
           double  v2_ls = U1.norm()*U1.norm();
//std::cout<<"the values="<<v2_ls<<std::endl;
Eigen::Vector3d Vec=V1-(U1*(V1.dot(U1)/v2_ls));
Eigen::Vector3d Vec1=Vec.normalized();
return Vec1;

}
   

   
Eigen::Matrix3d Rotation(Eigen::Vector3d u0,Eigen::Vector3d u1,Eigen::Vector3d v0,Eigen::Vector3d v1)
{
Eigen::Vector3d v00=V_proj(u0,v0);
//Eigen::Vector3d v0_0=v00.normalized();
//std::cout<<"prod="<<u0.dot(v0_0)<<std::endl;
Eigen::Vector3d s=u0.cross(v00); 
Eigen::Vector3d s0=s.normalized();
Eigen::Vector3d v11=V_proj(u1,v1);
//Eigen::Vector3d v1_1=v11.normalized();
Eigen::Vector3d q=u1.cross(v11);
//std::cout<<"prod2="<<u1.dot(v1_1)<<std::endl;
Eigen::Vector3d q0=q.normalized(); 
Eigen::Matrix3d M1;
M1.col(0)=u0;
M1.col(1)=v00;
M1.col(2)=s0;
Eigen::Matrix3d M2;
M2.col(0)=u1;
M2.col(1)=v11;
M2.col(2)=q0;
Eigen::Matrix3d R=M2*M1.inverse();
return R;
}

Line tr_line(Line l,  Eigen::Matrix4d  T)
     {

Line f;
Eigen::Vector4d A1=Eigen::Vector4d(l.A.x(), l.A.y(), l.A.z(),1);
Eigen::Vector4d B1=Eigen::Vector4d(l.B.x(), l.B.y(), l.B.z(),1);
Eigen::Vector4d C1=T.matrix()*A1;
Eigen::Vector4d C2=T.matrix()*B1;
Eigen::Vector3d A2=Eigen::Vector3d(C1.x(), C1.y(), C1.z());
Eigen::Vector3d B2=Eigen::Vector3d(C2.x(), C2.y(), C2.z());
f.A=A2;
f.B=B2;
f.d= direct_vec(A2,B2);
pcl::PointXYZ p1=pcl::PointXYZ(A2.x(), A2.y(), A2.z());
pcl::PointXYZ p2=pcl::PointXYZ(B2.x(), B2.y(), B2.z());
f.length=pcl::geometry::distance(p1,p2);
f.id_O=l.id_O;
f.id_pl=l.id_pl;
return f;
}

Line tr_line(Line l, Eigen::Matrix3d R)
     {

Line f;


Eigen::Vector3d A2=R*l.A;
Eigen::Vector3d B2=R*l.B;
f.A=A2;
f.B=B2;
f.d= direct_vec(A2,B2);
pcl::PointXYZ p1=pcl::PointXYZ(A2.x(), A2.y(),A2.z());
pcl::PointXYZ p2=pcl::PointXYZ(B2.x(), B2.y(), B2.z());
f.length=pcl::geometry::distance(p1,p2);
f.id_O=l.id_O;
f.id_pl=l.id_pl;
return f;

}























Eigen::Matrix4d pointToLine(std::vector< Eigen::Vector3d> src,std::vector<  Eigen::Vector3d> dst,std::vector<  Eigen::Vector3d> dir){
    assert(src.size()==dst.size() && src.size()==dir.size());
  
      Eigen::Matrix3d C; C.setZero();
    Eigen::Vector3d d; d.setZero();
Eigen::Matrix4d T = Eigen::Matrix4d::Identity();


        for(int i=0;i<src.size();++i){
       C-=vec_X(dir[i]).transpose()*vec_X(dir[i]);
        Eigen::Vector3d v=(src[i]-dst[i]);//
        
        // Eigen::Vector3d v=(dst[i]-src[i]);//
      d+=vec_X(dir[i]).transpose()*vec_X(dir[i])*v;
    }
    Eigen::Matrix<double,3,1> x = C.inverse()*d;
   T(0,3)=x(0);
   T(1,3)=x(1);
   T(2,3)=x(2);

    return T;
}
