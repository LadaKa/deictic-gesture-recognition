// Generated by gencpp from file pointing_gesture/BodyTracker.msg
// DO NOT EDIT!


#ifndef POINTING_GESTURE_MESSAGE__MESSAGE_BODYTRACKER_H
#define POINTING_GESTURE_MESSAGE__MESSAGE_BODYTRACKER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point32.h>

namespace pointing_gesture
{
template <class ContainerAllocator>
struct BodyTracker_
{     
  typedef BodyTracker_<ContainerAllocator> Type;

  BodyTracker_()
    : body_id(0)
    , tracking_status(0)
    , gesture(0)
    , face_found(false)
    , face_left(0)
    , face_top(0)
    , face_width(0)
    , face_height(0)
    , age(0)
    , gender(0)
    , name()
    , position2d()
    , position3d()
    , face_center()  {
    }
  BodyTracker_(const ContainerAllocator& _alloc)
    : body_id(0)
    , tracking_status(0)
    , gesture(0)
    , face_found(false)
    , face_left(0)
    , face_top(0)
    , face_width(0)
    , face_height(0)
    , age(0)
    , gender(0)
    , name(_alloc)
    , position2d(_alloc)
    , position3d(_alloc)
    , face_center(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _body_id_type;
  _body_id_type body_id;

   typedef int32_t _tracking_status_type;
  _tracking_status_type tracking_status;

   typedef int32_t _gesture_type;
  _gesture_type gesture;

   typedef uint8_t _face_found_type;
  _face_found_type face_found;

   typedef int32_t _face_left_type;
  _face_left_type face_left;

   typedef int32_t _face_top_type;
  _face_top_type face_top;

   typedef int32_t _face_width_type;
  _face_width_type face_width;

   typedef int32_t _face_height_type;
  _face_height_type face_height;

   typedef int32_t _age_type;
  _age_type age;

   typedef int32_t _gender_type;
  _gender_type gender;

  // typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  //_name_type name;

   std::string name;

   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _position2d_type;
  _position2d_type position2d;

   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _position3d_type;
  _position3d_type position3d;

   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _face_center_type;
  _face_center_type face_center;





  typedef boost::shared_ptr< ::pointing_gesture::BodyTracker_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pointing_gesture::BodyTracker_<ContainerAllocator> const> ConstPtr;

}; // struct BodyTracker_

typedef ::pointing_gesture::BodyTracker_<std::allocator<void> > BodyTracker;

typedef boost::shared_ptr< ::pointing_gesture::BodyTracker > BodyTrackerPtr;
typedef boost::shared_ptr< ::pointing_gesture::BodyTracker const> BodyTrackerConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pointing_gesture::BodyTracker_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pointing_gesture::BodyTracker_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pointing_gesture::BodyTracker_<ContainerAllocator1> & lhs, const ::pointing_gesture::BodyTracker_<ContainerAllocator2> & rhs)
{
  return lhs.body_id == rhs.body_id &&
    lhs.tracking_status == rhs.tracking_status &&
    lhs.gesture == rhs.gesture &&
    lhs.face_found == rhs.face_found &&
    lhs.face_left == rhs.face_left &&
    lhs.face_top == rhs.face_top &&
    lhs.face_width == rhs.face_width &&
    lhs.face_height == rhs.face_height &&
    lhs.age == rhs.age &&
    lhs.gender == rhs.gender &&
    lhs.name == rhs.name &&
    lhs.position2d == rhs.position2d &&
    lhs.position3d == rhs.position3d &&
    lhs.face_center == rhs.face_center;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pointing_gesture::BodyTracker_<ContainerAllocator1> & lhs, const ::pointing_gesture::BodyTracker_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pointing_gesture

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::pointing_gesture::BodyTracker_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pointing_gesture::BodyTracker_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pointing_gesture::BodyTracker_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pointing_gesture::BodyTracker_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pointing_gesture::BodyTracker_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pointing_gesture::BodyTracker_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pointing_gesture::BodyTracker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5fee6a28da28b41e53df055348e02173";
  }

  static const char* value(const ::pointing_gesture::BodyTracker_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5fee6a28da28b41eULL;
  static const uint64_t static_value2 = 0x53df055348e02173ULL;
};

template<class ContainerAllocator>
struct DataType< ::pointing_gesture::BodyTracker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pointing_gesture/BodyTracker";
  }

  static const char* value(const ::pointing_gesture::BodyTracker_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pointing_gesture::BodyTracker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32  body_id\n"
"int32  tracking_status\n"
"int32  gesture\n"
"bool   face_found\n"
"\n"
"# 2d face bounding Box position in pixels from 0,0 (top left of image)\n"
"int32  face_left\n"
"int32  face_top\n"
"int32  face_width\n"
"int32  face_height\n"
"int32  age     # rough estimate of persons age\n"
"int32  gender  # 0 = unknown, 1 = male, 2 = female\n"
"string name    # if match for persons face found in database\n"
"\n"
"geometry_msgs/Point32 position2d   # body x,y in camera frame, z = range from camera\n"
"geometry_msgs/Point32 position3d   # body x,y,z in world coordinates\n"
"geometry_msgs/Point32 face_center  # face x,y in camera frame, z = range from camera\n"
"\n"
"\n"
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

  static const char* value(const ::pointing_gesture::BodyTracker_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pointing_gesture::BodyTracker_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.body_id);
      stream.next(m.tracking_status);
      stream.next(m.gesture);
      stream.next(m.face_found);
      stream.next(m.face_left);
      stream.next(m.face_top);
      stream.next(m.face_width);
      stream.next(m.face_height);
      stream.next(m.age);
      stream.next(m.gender);
      stream.next(m.name);
      stream.next(m.position2d);
      stream.next(m.position3d);
      stream.next(m.face_center);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BodyTracker_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pointing_gesture::BodyTracker_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pointing_gesture::BodyTracker_<ContainerAllocator>& v)
  {
    s << indent << "body_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.body_id);
    s << indent << "tracking_status: ";
    Printer<int32_t>::stream(s, indent + "  ", v.tracking_status);
    s << indent << "gesture: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gesture);
    s << indent << "face_found: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.face_found);
    s << indent << "face_left: ";
    Printer<int32_t>::stream(s, indent + "  ", v.face_left);
    s << indent << "face_top: ";
    Printer<int32_t>::stream(s, indent + "  ", v.face_top);
    s << indent << "face_width: ";
    Printer<int32_t>::stream(s, indent + "  ", v.face_width);
    s << indent << "face_height: ";
    Printer<int32_t>::stream(s, indent + "  ", v.face_height);
    s << indent << "age: ";
    Printer<int32_t>::stream(s, indent + "  ", v.age);
    s << indent << "gender: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gender);
    //s << indent << "name: ";
    //Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "position2d: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.position2d);
    s << indent << "position3d: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.position3d);
    s << indent << "face_center: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.face_center);
  }
};

} // namespace message_operations
} // namespace ros

#endif // pointing_gesture_MESSAGE_BODYTRACKER_H
