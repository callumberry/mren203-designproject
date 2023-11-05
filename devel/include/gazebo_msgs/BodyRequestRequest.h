// Generated by gencpp from file gazebo_msgs/BodyRequestRequest.msg
// DO NOT EDIT!


#ifndef GAZEBO_MSGS_MESSAGE_BODYREQUESTREQUEST_H
#define GAZEBO_MSGS_MESSAGE_BODYREQUESTREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace gazebo_msgs
{
template <class ContainerAllocator>
struct BodyRequestRequest_
{
  typedef BodyRequestRequest_<ContainerAllocator> Type;

  BodyRequestRequest_()
    : body_name()  {
    }
  BodyRequestRequest_(const ContainerAllocator& _alloc)
    : body_name(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _body_name_type;
  _body_name_type body_name;





  typedef boost::shared_ptr< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> const> ConstPtr;

}; // struct BodyRequestRequest_

typedef ::gazebo_msgs::BodyRequestRequest_<std::allocator<void> > BodyRequestRequest;

typedef boost::shared_ptr< ::gazebo_msgs::BodyRequestRequest > BodyRequestRequestPtr;
typedef boost::shared_ptr< ::gazebo_msgs::BodyRequestRequest const> BodyRequestRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator1> & lhs, const ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator2> & rhs)
{
  return lhs.body_name == rhs.body_name;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator1> & lhs, const ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace gazebo_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5eade9afe7f232d78005bd0cafeab755";
  }

  static const char* value(const ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5eade9afe7f232d7ULL;
  static const uint64_t static_value2 = 0x8005bd0cafeab755ULL;
};

template<class ContainerAllocator>
struct DataType< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gazebo_msgs/BodyRequestRequest";
  }

  static const char* value(const ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string body_name   # name of the body requested. body names are prefixed by model name, e.g. pr2::base_link\n"
;
  }

  static const char* value(const ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.body_name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BodyRequestRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gazebo_msgs::BodyRequestRequest_<ContainerAllocator>& v)
  {
    s << indent << "body_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.body_name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GAZEBO_MSGS_MESSAGE_BODYREQUESTREQUEST_H
