//-------------------------------------------------------------------------------------------
/*! \file    geom_util.h
    \brief   Geometry utilities
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.18, 2015
*/
//-------------------------------------------------------------------------------------------
#ifndef geom_util_h
#define geom_util_h
//-------------------------------------------------------------------------------------------
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
// Basic geometory utilities.
//-------------------------------------------------------------------------------------------

// For STL containers:
struct TPos {double X[3]; /*[0-2]: position x,y,z.*/};
struct TPose {double X[7]; /*[0-2]: position x,y,z, [3-6]: orientation qx,qy,qz,qw.*/};
struct TPosVel {double X[6]; /*[0-2]: position x,y,z, [3-5]: velocity vx,vy,vz.*/};
//-------------------------------------------------------------------------------------------

/* Pose to Eigen::Affine3d.
    x: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline Eigen::Affine3d XToEigMat(const t_value x[7])
{
  Eigen::Affine3d res;
  res= Eigen::Translation3d(x[0],x[1],x[2])
          * Eigen::Quaterniond(x[6], x[3],x[4],x[5]);
  return res;
}
//-------------------------------------------------------------------------------------------

/* Orientation to Eigen::Quaterniond.
    x: [0-3]: orientation x,y,z,w. */
template <typename t_value>
inline Eigen::Quaterniond QToEigMat(const t_value x[4])
{
  return Eigen::Quaterniond(x[3], x[0],x[1],x[2]);
}
//-------------------------------------------------------------------------------------------

/* Eigen::Affine3d to position.
    x: [0-2]: position x,y,z. */
template <typename t_value>
inline void EigMatToP(const Eigen::Affine3d T, t_value x[3])
{
  const Eigen::Vector3d p= T.translation();
  x[0]= p[0];
  x[1]= p[1];
  x[2]= p[2];
}
//-------------------------------------------------------------------------------------------

/* Eigen::Quaterniond to orientation.
    x: [0-3]: orientation x,y,z,w. */
template <typename t_value>
inline void EigMatToQ(const Eigen::Quaterniond q, t_value x[4])
{
  x[0]= q.x();
  x[1]= q.y();
  x[2]= q.z();
  x[3]= q.w();
}
/* Eigen::Affine3d to orientation.
    x: [0-3]: orientation x,y,z,w. */
template <typename t_value>
inline void EigMatToQ(const Eigen::Affine3d T, t_value x[4])
{
  EigMatToQ(T.rotation(), x);
}
//-------------------------------------------------------------------------------------------

/* Eigen::Affine3d to pose.
    x: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline void EigMatToX(const Eigen::Affine3d T, t_value x[7])
{
  const Eigen::Vector3d p= T.translation();
  x[0]= p[0];
  x[1]= p[1];
  x[2]= p[2];
  Eigen::Quaterniond q(T.rotation());
  x[3]= q.x();
  x[4]= q.y();
  x[5]= q.z();
  x[6]= q.w();
}
//-------------------------------------------------------------------------------------------

/* Compute xout = x2 * x1
    where x1,x2,xout: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline void TransformX(const t_value x2[7], const t_value x1[7], t_value xout[7])
{
  EigMatToX(XToEigMat(x2)*XToEigMat(x1), xout);
}
//-------------------------------------------------------------------------------------------

/* Compute xout = x2 * p1
    where p1,xout: [0-2]: position x,y,z,
      x2: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_value>
inline void TransformP(const t_value x2[7], const t_value p1[3], t_value xout[3])
{
  EigMatToP(XToEigMat(x2)*Eigen::Translation3d(p1[0],p1[1],p1[2]), xout);
}
//-------------------------------------------------------------------------------------------

/* Rotate q_in with quaternion(angle,axis) then store to q_out.
    where q_in,q_out: [0-3]: orientation x,y,z,w,
      axis: [0-2]: axis x,y,z. */
template <typename t_value>
inline void RotateAngleAxis(const t_value &angle, const t_value axis[3], const t_value q_in[4], t_value q_out[4])
{
  Eigen::Quaterniond q= Eigen::AngleAxisd(angle, Eigen::Vector3d(axis)) * QToEigMat(q_in);
  EigMatToQ(q, q_out);
}
//-------------------------------------------------------------------------------------------

/* Return axis*angle with which v1 is rotated to v2's direction.
    axis_angle: axis*angle (i.e. its norm is angle). */
template <typename t_value, typename t_array>
inline void GetAxisAngle(const t_value v1[3], const t_value v2[3], t_array &axis_angle)
{
  typedef Eigen::Matrix<t_value,3,1> Vector3;
  Vector3 p1(v1), p2(v2);
  Vector3 p1xp2= p1.cross(p2);
  double p1xp2_norm= p1xp2.norm();
  if(p1xp2_norm<1.0e-8)
    {for(int d(0);d<3;++d) axis_angle[d]=0.0; return;}
  Vector3 axis= p1xp2/p1xp2_norm;
  Vector3 ex= p1.normalized();
  Vector3 ey= axis.cross(ex);
  double angle= std::atan2(ey.dot(p2), ex.dot(p2));
  axis*= angle;
  for(int d(0);d<3;++d)  axis_angle[d]= axis[d];
}
//-------------------------------------------------------------------------------------------

// (Ported from ay_py.core.geom)
// Orthogonalize a vector vec w.r.t. base; i.e. vec is modified so that dot(vec,base)==0.
// original_norm: keep original vec's norm, otherwise the norm is 1.
// Using The Gram-Schmidt process: http://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
template <typename t_value>
inline void Orthogonalize(const t_value vec[3], const t_value base[3], t_value out[3], bool original_norm=true)
{
  typedef Eigen::Matrix<t_value,3,1> Vector3;
  Vector3 v(vec);
  Vector3 vbase= Vector3(base).normalized();
  Vector3 v2= v - v.dot(vbase)*vbase;
  Eigen::Map<Vector3> vout(out);
  vout= (original_norm ? (v2.normalized()*v.norm()) : v2.normalized());
}
//-------------------------------------------------------------------------------------------

// (Ported from ay_py.core.geom)
// Get an orthogonal axis of a given axis
// preferable: preferable axis (orthogonal axis is close to this)
// fault: return this axis when dot(axis,preferable)==1
template <typename t_value>
inline void GetOrthogonalAxisOf(const t_value axis[3], t_value out[3], const t_value preferable[3]=NULL, const t_value fault[]=NULL)
{
  static const t_value default_preferable[3]= {0.0,0.0,1.0};
  if(preferable==NULL)  preferable= default_preferable;
  typedef Eigen::Matrix<t_value,3,1> Vector3;
  t_value naxis[3];
  Eigen::Map<Vector3> vnaxis(naxis);
  vnaxis= Vector3(axis).normalized();

  if(fault==NULL || 1.0-std::fabs(vnaxis.dot(Vector3(preferable)))>=1.0e-6)
    Orthogonalize(preferable, /*base=*/naxis, out, /*original_norm=*/false);
  else
  {
    Eigen::Map<Vector3> vout(out);
    vout= Vector3(fault);
  }
}
//-------------------------------------------------------------------------------------------

// (Ported from ay_py.core.geom)
// For visualizing cylinder, arrow, etc., get a pose x from two points p1-->p2.
// Axis ax decides which axis corresponds to p1-->p2.
// Ratio r decides: r=0: x is on p1, r=1: x is on p2, r=0.5: x is on the middle of p1 and p2.
template <typename t_value>
inline void XFromP1P2(const t_value p1[3], const t_value p2[3], t_value x_out[7], char ax='z', const t_value &r=0.5)
{
  typedef Eigen::Matrix<t_value,3,1> Vector3;
  typedef Eigen::Matrix<t_value,4,1> Vector4;
  typedef Eigen::Matrix<t_value,7,1> Vector7;
  Vector3 v1(p1), v2(p2);
  t_value aex[3],aey[3],aez[3];
  Eigen::Map<Vector3> ex(aex),ey(aey),ez(aez);
  if(ax=='x')
  {
    ex= (v2-v1).normalized();
    t_value preferable[]={0.0,1.0,0.0}, fault[]={0.0,0.0,1.0};
    GetOrthogonalAxisOf(aex, /*out=*/aey, preferable, fault);
    ez= ex.cross(ey);
  }
  else if(ax=='y')
  {
    ey= (v2-v1).normalized();
    t_value preferable[]={0.0,0.0,1.0}, fault[]={1.0,0.0,0.0};
    GetOrthogonalAxisOf(aey, /*out=*/aez, preferable, fault);
    ex= ey.cross(ez);
  }
  else if(ax=='z')
  {
    ez= (v2-v1).normalized();
    t_value preferable[]={1.0,0.0,0.0}, fault[]={0.0,1.0,0.0};
    GetOrthogonalAxisOf(aez, /*out=*/aex, preferable, fault);
    ey= ez.cross(ex);
  }
  Eigen::Map<Vector7> x(x_out);
  x.segment(0,3)= (1.0-r)*v1+r*v2;
  Eigen::Matrix<t_value,3,3> rot;
  rot<<ex,ey,ez;
  Eigen::Quaternion<t_value> q(rot);
  x.segment(3,4)= Vector4(q.x(),q.y(),q.z(),q.w());
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
// Utility for ROS::geometry_msgs
//-------------------------------------------------------------------------------------------

// Convert p to geometry_msgs/Point; usually, t_point==geometry_msgs::Point
template <typename t_array, typename t_point>
inline void PToGPoint(const t_array p, t_point &point)
{
  point.x= p[0];
  point.y= p[1];
  point.z= p[2];
}
// Convert p to geometry_msgs/Point; usually, t_point==geometry_msgs::Point
template <typename t_point, typename t_array>
inline t_point PToGPoint(const t_array p)
{
  t_point point;
  PToGPoint<t_array,t_point>(p, point);
  return point;
}
//-------------------------------------------------------------------------------------------

// Convert a vector of p to a vector of geometry_msgs/Point; usually, t_point==geometry_msgs::Point
template <typename t_array, typename t_point>
inline void PToGPointVector(const std::vector<t_array> ps, std::vector<t_point> &points)
{
  points.resize(ps.size());
  for(int i(0),i_end(ps.size()); i<i_end; ++i)
    PToGPoint(ps[i], points[i]);
}
// Convert a vector of p to a vector of geometry_msgs/Point; usually, t_point==geometry_msgs::Point
template <typename t_point, typename t_array>
inline std::vector<t_point> PToGPointVector(const std::vector<t_array> ps)
{
  std::vector<t_point> points(ps.size());
  PToGPointVector<t_array,t_point>(ps, points);
  return points;
}
//-------------------------------------------------------------------------------------------

// Convert geometry_msgs/Point to p; usually, t_point==geometry_msgs::Point
template <typename t_point, typename t_array>
inline void GPointToP(const t_point &point, t_array p)
{
  p[0]= point.x;
  p[1]= point.y;
  p[2]= point.z;
}
//-------------------------------------------------------------------------------------------

// Convert x to geometry_msgs/Pose; usually, t_pose==geometry_msgs::Pose
template <typename t_array, typename t_pose>
inline void XToGPose(const t_array x, t_pose &pose)
{
  pose.position.x= x[0];
  pose.position.y= x[1];
  pose.position.z= x[2];
  pose.orientation.x= x[3];
  pose.orientation.y= x[4];
  pose.orientation.z= x[5];
  pose.orientation.w= x[6];
}
// Convert x to geometry_msgs/Pose; usually, t_pose==geometry_msgs::Pose
template <typename t_pose, typename t_array>
inline t_pose XToGPose(const t_array x)
{
  t_pose pose;
  XToGPose<t_array,t_pose>(x, pose);
  return pose;
}
//-------------------------------------------------------------------------------------------

// Convert a vector of x to a vector of geometry_msgs/Pose; usually, t_pose==geometry_msgs::Pose
template <typename t_array, typename t_pose>
inline void XToGPoseVector(const std::vector<t_array> xs, std::vector<t_pose> &poses)
{
  poses.resize(xs.size());
  for(int i(0),i_end(xs.size()); i<i_end; ++i)
    XToGPose(xs[i], poses[i]);
}
// Convert a vector of x to a vector of geometry_msgs/Pose; usually, t_pose==geometry_msgs::Pose
template <typename t_pose, typename t_array>
inline std::vector<t_pose> XToGPoseVector(const std::vector<t_array> xs)
{
  std::vector<t_pose> poses(xs.size());
  XToGPoseVector<t_array,t_pose>(xs, poses);
  return poses;
}
//-------------------------------------------------------------------------------------------

// Convert geometry_msgs/Pose to x; usually, t_pose==geometry_msgs::Pose
template <typename t_pose, typename t_array>
inline void GPoseToX(const t_pose &pose, t_array x)
{
  x[0]= pose.position.x;
  x[1]= pose.position.y;
  x[2]= pose.position.z;
  x[3]= pose.orientation.x;
  x[4]= pose.orientation.y;
  x[5]= pose.orientation.z;
  x[6]= pose.orientation.w;
}
//-------------------------------------------------------------------------------------------

// t_point: e.g. geometry_msgs::Point, Vector3
template<typename t_point, typename t_value=double>
inline t_point GenGPoint(const t_value &x=0.0, const t_value &y=0.0, const t_value &z=0.0)
{
  t_point p;
  p.x= x;
  p.y= y;
  p.z= z;
  return p;
}
//-------------------------------------------------------------------------------------------
// t_quaternion: e.g. geometry_msgs::Quaternion
template<typename t_quaternion, typename t_value=double>
inline t_quaternion GenGQuaternion(const t_value &x=0.0, const t_value &y=0.0, const t_value &z=0.0, const t_value &w=0.0)
{
  t_quaternion p;
  p.x= x;
  p.y= y;
  p.z= z;
  p.w= w;
  return p;
}
//-------------------------------------------------------------------------------------------
// t_rgba: e.g. geometry_msgs::ColorRGBA
template<typename t_rgba, typename t_value_r=double, typename t_value_g=double, typename t_value_b=double, typename t_value_a=double>
inline t_rgba GenGRBGA(const t_value_r &r=1.0, const t_value_g &g=1.0, const t_value_b &b=1.0, const t_value_a &a=1.0)
{
  t_rgba c;
  c.r= r;
  c.g= g;
  c.b= b;
  c.a= a;
  return c;
}
//-------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------
// Advanced geometory utilities.
//-------------------------------------------------------------------------------------------


/* Find a least square solution of W in W*x=y for {(x,y)}.
    x= (x_1,...,x_d); d: dim of x.
    y= (y_1,...,y_c); c: dim of y.
    X= [x_1_1,...,x_d_1]
       [x_1_2,...,x_d_2]
       ...
       [x_1_N,...,x_d_N]; N: num of samples.
    Y= [y_1_1,...,y_c_1]
       [y_1_2,...,y_c_2]
       ...
       [y_1_N,...,y_c_N]; N: num of samples.
    lambda: regularization parameter.
    return W.
*/
inline Eigen::MatrixXd LeastSq(const Eigen::MatrixXd &X, const Eigen::MatrixXd &Y, const double &lambda=0.01)
{
  assert(X.rows()==Y.rows());
  Eigen::MatrixXd I= Eigen::MatrixXd::Identity(X.cols(),X.cols());
  return /*W=*/ ( (X.transpose()*X + lambda * I).inverse()*X.transpose() * Y ).transpose();
}
//-------------------------------------------------------------------------------------------


// Visualize a normal vector with RGB color.
template <typename t_1, typename t_2>
inline void GetVisualNormal(
    const t_1 &nx, const t_1 &ny, const t_1 &nz,
    t_2 &r, t_2 &g, t_2 &b)
{
  // Version 1
  // r= 0.5*(1.0-nx);
  // g= 0.5*(1.0-ny);
  // b= 0.5*(1.0-nz);
  // Version 2 (consider [nx,ny,nz]==[-nx,-ny,-nz])
  if(nz>=0.0)
  {
    r= 0.5*(1.0-nx);
    g= 0.5*(1.0-ny);
    b= 0.5*(1.0-nz);
  }
  else
  {
    r= 0.5*(1.0+nx);
    g= 0.5*(1.0+ny);
    b= 0.5*(1.0+nz);
  }
}
//-------------------------------------------------------------------------------------------

template <typename t_value=double, bool using_minmax=false>
struct TRotatedBoundingBox
{
  t_value X[7];  // Center pose: x,y,z, qx,qy,qz,qw
  t_value Size[3];  // x-size,y-size,z-size; used if not using_minmax
  t_value Min[3], Max[3];  // min and max of x,y,z; used if using_minmax

  TRotatedBoundingBox()  {Init();}

  void Init()
    {
      for(int d(0);d<7;++d)  X[d]= 0.0;
      X[6]= 1.0;  // qw
      for(int d(0);d<3;++d)  Size[d]= 0.0;
    }

  // Check if a given point (x,y,z) is inside the bounding box.
  //   inv_x: Use the result of InvX() for speed up when evaluating many points.
  bool IsIn(const t_value p[3], Eigen::Affine3d *inv_x=NULL) const
    {
      // Compute a position of p in this BB's local frame
      t_value l_p[3];
      if(inv_x==NULL)
        EigMatToP(XToEigMat(X).inverse() * Eigen::Translation3d(p[0],p[1],p[2]), l_p);
      else
        EigMatToP((*inv_x) * Eigen::Translation3d(p[0],p[1],p[2]), l_p);
      // Check if l_p is inside the AABB
      if(using_minmax)  // using Min,Max
      {
        for(int d(0);d<3;++d)
          if(l_p[d]<Min[d] || l_p[d]>Max[d])
            return false;
      }
      else  // using Size
      {
        for(int d(0);d<3;++d)
          if(l_p[d]<-0.5*Size[d] || l_p[d]>+0.5*Size[d])
            return false;
      }
      return true;
    }

  Eigen::Affine3d InvX() const {return XToEigMat(X).inverse();}
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // geom_util_h
//-------------------------------------------------------------------------------------------
