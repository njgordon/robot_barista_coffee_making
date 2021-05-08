// Generated by gencpp from file manipulation/ReturnJointStatesResponse.msg
// DO NOT EDIT!


#ifndef MANIPULATION_MESSAGE_RETURNJOINTSTATESRESPONSE_H
#define MANIPULATION_MESSAGE_RETURNJOINTSTATESRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace manipulation
{
template <class ContainerAllocator>
struct ReturnJointStatesResponse_
{
  typedef ReturnJointStatesResponse_<ContainerAllocator> Type;

  ReturnJointStatesResponse_()
    : found()
    , position()
    , velocity()
    , effort()  {
    }
  ReturnJointStatesResponse_(const ContainerAllocator& _alloc)
    : found(_alloc)
    , position(_alloc)
    , velocity(_alloc)
    , effort(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  _found_type;
  _found_type found;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _position_type;
  _position_type position;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _velocity_type;
  _velocity_type velocity;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _effort_type;
  _effort_type effort;





  typedef boost::shared_ptr< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ReturnJointStatesResponse_

typedef ::manipulation::ReturnJointStatesResponse_<std::allocator<void> > ReturnJointStatesResponse;

typedef boost::shared_ptr< ::manipulation::ReturnJointStatesResponse > ReturnJointStatesResponsePtr;
typedef boost::shared_ptr< ::manipulation::ReturnJointStatesResponse const> ReturnJointStatesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::manipulation::ReturnJointStatesResponse_<ContainerAllocator1> & lhs, const ::manipulation::ReturnJointStatesResponse_<ContainerAllocator2> & rhs)
{
  return lhs.found == rhs.found &&
    lhs.position == rhs.position &&
    lhs.velocity == rhs.velocity &&
    lhs.effort == rhs.effort;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::manipulation::ReturnJointStatesResponse_<ContainerAllocator1> & lhs, const ::manipulation::ReturnJointStatesResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace manipulation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3a36649f5b1439b638a41d18af93e9a4";
  }

  static const char* value(const ::manipulation::ReturnJointStatesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3a36649f5b1439b6ULL;
  static const uint64_t static_value2 = 0x38a41d18af93e9a4ULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "manipulation/ReturnJointStatesResponse";
  }

  static const char* value(const ::manipulation::ReturnJointStatesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32[] found\n"
"float64[] position\n"
"float64[] velocity\n"
"float64[] effort\n"
;
  }

  static const char* value(const ::manipulation::ReturnJointStatesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.found);
      stream.next(m.position);
      stream.next(m.velocity);
      stream.next(m.effort);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ReturnJointStatesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::manipulation::ReturnJointStatesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::manipulation::ReturnJointStatesResponse_<ContainerAllocator>& v)
  {
    s << indent << "found[]" << std::endl;
    for (size_t i = 0; i < v.found.size(); ++i)
    {
      s << indent << "  found[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.found[i]);
    }
    s << indent << "position[]" << std::endl;
    for (size_t i = 0; i < v.position.size(); ++i)
    {
      s << indent << "  position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position[i]);
    }
    s << indent << "velocity[]" << std::endl;
    for (size_t i = 0; i < v.velocity.size(); ++i)
    {
      s << indent << "  velocity[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.velocity[i]);
    }
    s << indent << "effort[]" << std::endl;
    for (size_t i = 0; i < v.effort.size(); ++i)
    {
      s << indent << "  effort[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.effort[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MANIPULATION_MESSAGE_RETURNJOINTSTATESRESPONSE_H
