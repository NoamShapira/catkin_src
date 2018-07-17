#include <ros/ros.h>
#include <tf/transform_listener.h>
// #include <tf/tfMessage>
#include <string>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle nh;
  tf::TransformListener listener;

  ros::Rate rate(1.0);
  
  while(nh.ok()){
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("cam1_link", "base_link",  
                               ros::Time(0), transform);

      tf::Matrix3x3 rotation = transform.getBasis();
      tf::Vector3 translation = transform.getOrigin();
      std::cout<<"child_link: "<<transform.child_frame_id_<<"\n";
      
      std::cout<<"x: "<<translation.x();
      std::cout<<" ,y: "<<translation.y();
      std::cout<<" ,z: "<<translation.z()<<"\n";

      // translation.distance()

      tf::Transform invers_tf = transform.inverse();
      std::cout<<"x: "<<invers_tf.getOrigin().x();
      std::cout<<" ,y: "<<invers_tf.getOrigin().y();
      std::cout<<" ,z: "<<invers_tf.getOrigin().z()<<"\n\n";
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
    
  }

}