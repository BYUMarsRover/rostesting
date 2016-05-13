// Generated by gencpp from file rover_msgs/JointAngles.msg
// DO NOT EDIT!


#ifndef ROVER_MSGS_MESSAGE_JOINTANGLES_H
#define ROVER_MSGS_MESSAGE_JOINTANGLES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rover_msgs
{
template <class ContainerAllocator>
struct JointAngles_
{
  typedef JointAngles_<ContainerAllocator> Type;

  JointAngles_()
    : q()
    , solved(0)  {
    }
  JointAngles_(const ContainerAllocator& _alloc)
    : q(_alloc)
    , solved(0)  {
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _q_type;
  _q_type q;

   typedef int16_t _solved_type;
  _solved_type solved;




  typedef boost::shared_ptr< ::rover_msgs::JointAngles_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rover_msgs::JointAngles_<ContainerAllocator> const> ConstPtr;

}; // struct JointAngles_

typedef ::rover_msgs::JointAngles_<std::allocator<void> > JointAngles;

typedef boost::shared_ptr< ::rover_msgs::JointAngles > JointAnglesPtr;
typedef boost::shared_ptr< ::rover_msgs::JointAngles const> JointAnglesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rover_msgs::JointAngles_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rover_msgs::JointAngles_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rover_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'rover_msgs': ['/home/halrover/git/rostesting/rover_ws/src/rover_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rover_msgs::JointAngles_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rover_msgs::JointAngles_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rover_msgs::JointAngles_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rover_msgs::JointAngles_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rover_msgs::JointAngles_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rover_msgs::JointAngles_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rover_msgs::JointAngles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "58290761aec98e324187e922e595db5b";
  }

  static const char* value(const ::rover_msgs::JointAngles_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x58290761aec98e32ULL;
  static const uint64_t static_value2 = 0x4187e922e595db5bULL;
};

template<class ContainerAllocator>
struct DataType< ::rover_msgs::JointAngles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rover_msgs/JointAngles";
  }

  static const char* value(const ::rover_msgs::JointAngles_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rover_msgs::JointAngles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] q\n\
int16 solved\n\
";
  }

  static const char* value(const ::rover_msgs::JointAngles_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rover_msgs::JointAngles_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.q);
      stream.next(m.solved);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct JointAngles_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rover_msgs::JointAngles_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rover_msgs::JointAngles_<ContainerAllocator>& v)
  {
    s << indent << "q[]" << std::endl;
    for (size_t i = 0; i < v.q.size(); ++i)
    {
      s << indent << "  q[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.q[i]);
    }
    s << indent << "solved: ";
    Printer<int16_t>::stream(s, indent + "  ", v.solved);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROVER_MSGS_MESSAGE_JOINTANGLES_H
