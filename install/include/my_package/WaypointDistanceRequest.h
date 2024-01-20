// Generated by gencpp from file my_package/WaypointDistanceRequest.msg
// DO NOT EDIT!


#ifndef MY_PACKAGE_MESSAGE_WAYPOINTDISTANCEREQUEST_H
#define MY_PACKAGE_MESSAGE_WAYPOINTDISTANCEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace my_package
{
template <class ContainerAllocator>
struct WaypointDistanceRequest_
{
  typedef WaypointDistanceRequest_<ContainerAllocator> Type;

  WaypointDistanceRequest_()
    : robot_id(0)
    , waypoint_id(0)  {
    }
  WaypointDistanceRequest_(const ContainerAllocator& _alloc)
    : robot_id(0)
    , waypoint_id(0)  {
  (void)_alloc;
    }



   typedef uint8_t _robot_id_type;
  _robot_id_type robot_id;

   typedef uint8_t _waypoint_id_type;
  _waypoint_id_type waypoint_id;





  typedef boost::shared_ptr< ::my_package::WaypointDistanceRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::my_package::WaypointDistanceRequest_<ContainerAllocator> const> ConstPtr;

}; // struct WaypointDistanceRequest_

typedef ::my_package::WaypointDistanceRequest_<std::allocator<void> > WaypointDistanceRequest;

typedef boost::shared_ptr< ::my_package::WaypointDistanceRequest > WaypointDistanceRequestPtr;
typedef boost::shared_ptr< ::my_package::WaypointDistanceRequest const> WaypointDistanceRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::my_package::WaypointDistanceRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::my_package::WaypointDistanceRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::my_package::WaypointDistanceRequest_<ContainerAllocator1> & lhs, const ::my_package::WaypointDistanceRequest_<ContainerAllocator2> & rhs)
{
  return lhs.robot_id == rhs.robot_id &&
    lhs.waypoint_id == rhs.waypoint_id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::my_package::WaypointDistanceRequest_<ContainerAllocator1> & lhs, const ::my_package::WaypointDistanceRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace my_package

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::my_package::WaypointDistanceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_package::WaypointDistanceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_package::WaypointDistanceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_package::WaypointDistanceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_package::WaypointDistanceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_package::WaypointDistanceRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::my_package::WaypointDistanceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "831c18e1bdee664e0ac4ccc25433d3c2";
  }

  static const char* value(const ::my_package::WaypointDistanceRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x831c18e1bdee664eULL;
  static const uint64_t static_value2 = 0x0ac4ccc25433d3c2ULL;
};

template<class ContainerAllocator>
struct DataType< ::my_package::WaypointDistanceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "my_package/WaypointDistanceRequest";
  }

  static const char* value(const ::my_package::WaypointDistanceRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::my_package::WaypointDistanceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8    robot_id\n"
"uint8    waypoint_id\n"
;
  }

  static const char* value(const ::my_package::WaypointDistanceRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::my_package::WaypointDistanceRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.robot_id);
      stream.next(m.waypoint_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WaypointDistanceRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::my_package::WaypointDistanceRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::my_package::WaypointDistanceRequest_<ContainerAllocator>& v)
  {
    s << indent << "robot_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.robot_id);
    s << indent << "waypoint_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.waypoint_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MY_PACKAGE_MESSAGE_WAYPOINTDISTANCEREQUEST_H