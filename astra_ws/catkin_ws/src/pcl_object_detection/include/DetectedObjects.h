// Generated by gencpp from file pcl_object_detection/DetectedObjects.msg
// DO NOT EDIT!


#ifndef PCL_OBJECT_DETECTION_MESSAGE_DETECTEDOBJECTS_H
#define PCL_OBJECT_DETECTION_MESSAGE_DETECTEDOBJECTS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point32.h>

namespace pcl_object_detection
{
template <class ContainerAllocator>
struct DetectedObjects_
{
  typedef DetectedObjects_<ContainerAllocator> Type;

  DetectedObjects_()
    : objectsCenters()  {
    }
  DetectedObjects_(const ContainerAllocator& _alloc)
    : objectsCenters()  {
  (void)_alloc;
      objectsCenters.assign( ::geometry_msgs::Point32_<ContainerAllocator> (_alloc));
  }



   typedef boost::array< ::geometry_msgs::Point32_<ContainerAllocator> , 3>  _objectsCenters_type;
  _objectsCenters_type objectsCenters;





  typedef boost::shared_ptr< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> const> ConstPtr;

}; // struct DetectedObjects_

typedef ::pcl_object_detection::DetectedObjects_<std::allocator<void> > DetectedObjects;

typedef boost::shared_ptr< ::pcl_object_detection::DetectedObjects > DetectedObjectsPtr;
typedef boost::shared_ptr< ::pcl_object_detection::DetectedObjects const> DetectedObjectsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pcl_object_detection::DetectedObjects_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pcl_object_detection::DetectedObjects_<ContainerAllocator1> & lhs, const ::pcl_object_detection::DetectedObjects_<ContainerAllocator2> & rhs)
{
  return lhs.objectsCenters == rhs.objectsCenters;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pcl_object_detection::DetectedObjects_<ContainerAllocator1> & lhs, const ::pcl_object_detection::DetectedObjects_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pcl_object_detection

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> >
{
  static const char* value()
  {
    return "17094eefbe3742486c3f60ffb5186309";
  }

  static const char* value(const ::pcl_object_detection::DetectedObjects_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x17094eefbe374248ULL;
  static const uint64_t static_value2 = 0x6c3f60ffb5186309ULL;
};

template<class ContainerAllocator>
struct DataType< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pcl_object_detection/DetectedObjects";
  }

  static const char* value(const ::pcl_object_detection::DetectedObjects_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point32[3] objectsCenters\n"
"================================================================================\n"
"MSG: geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::pcl_object_detection::DetectedObjects_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.objectsCenters);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DetectedObjects_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pcl_object_detection::DetectedObjects_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pcl_object_detection::DetectedObjects_<ContainerAllocator>& v)
  {
    s << indent << "objectsCenters[]" << std::endl;
    for (size_t i = 0; i < v.objectsCenters.size(); ++i)
    {
      s << indent << "  objectsCenters[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "    ", v.objectsCenters[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PCL_OBJECT_DETECTION_MESSAGE_DETECTEDOBJECTS_H
