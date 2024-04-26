// Generated by gencpp from file ublox_msgs/NavPOSECEF.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_NAVPOSECEF_H
#define UBLOX_MSGS_MESSAGE_NAVPOSECEF_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ublox_msgs
{
template <class ContainerAllocator>
struct NavPOSECEF_
{
  typedef NavPOSECEF_<ContainerAllocator> Type;

  NavPOSECEF_()
    : iTOW(0)
    , ecefX(0)
    , ecefY(0)
    , ecefZ(0)
    , pAcc(0)  {
    }
  NavPOSECEF_(const ContainerAllocator& _alloc)
    : iTOW(0)
    , ecefX(0)
    , ecefY(0)
    , ecefZ(0)
    , pAcc(0)  {
  (void)_alloc;
    }



   typedef uint32_t _iTOW_type;
  _iTOW_type iTOW;

   typedef int32_t _ecefX_type;
  _ecefX_type ecefX;

   typedef int32_t _ecefY_type;
  _ecefY_type ecefY;

   typedef int32_t _ecefZ_type;
  _ecefZ_type ecefZ;

   typedef uint32_t _pAcc_type;
  _pAcc_type pAcc;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(CLASS_ID)
  #undef CLASS_ID
#endif
#if defined(_WIN32) && defined(MESSAGE_ID)
  #undef MESSAGE_ID
#endif

  enum {
    CLASS_ID = 1u,
    MESSAGE_ID = 1u,
  };


  typedef boost::shared_ptr< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> const> ConstPtr;

}; // struct NavPOSECEF_

typedef ::ublox_msgs::NavPOSECEF_<std::allocator<void> > NavPOSECEF;

typedef boost::shared_ptr< ::ublox_msgs::NavPOSECEF > NavPOSECEFPtr;
typedef boost::shared_ptr< ::ublox_msgs::NavPOSECEF const> NavPOSECEFConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::NavPOSECEF_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ublox_msgs::NavPOSECEF_<ContainerAllocator1> & lhs, const ::ublox_msgs::NavPOSECEF_<ContainerAllocator2> & rhs)
{
  return lhs.iTOW == rhs.iTOW &&
    lhs.ecefX == rhs.ecefX &&
    lhs.ecefY == rhs.ecefY &&
    lhs.ecefZ == rhs.ecefZ &&
    lhs.pAcc == rhs.pAcc;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ublox_msgs::NavPOSECEF_<ContainerAllocator1> & lhs, const ::ublox_msgs::NavPOSECEF_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6f1f4f9473d5586f7fa1427a3c53cee9";
  }

  static const char* value(const ::ublox_msgs::NavPOSECEF_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6f1f4f9473d5586fULL;
  static const uint64_t static_value2 = 0x7fa1427a3c53cee9ULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/NavPOSECEF";
  }

  static const char* value(const ::ublox_msgs::NavPOSECEF_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# NAV-POSECEF (0x01 0x01)\n"
"# Position Solution in ECEF\n"
"#\n"
"# See important comments concerning validity of position given in section\n"
"# Navigation Output Filters.\n"
"#\n"
"\n"
"uint8 CLASS_ID = 1\n"
"uint8 MESSAGE_ID = 1\n"
"\n"
"uint32 iTOW             # GPS Millisecond Time of Week [ms]\n"
"\n"
"int32 ecefX             # ECEF X coordinate [cm]\n"
"int32 ecefY             # ECEF Y coordinate [cm]\n"
"int32 ecefZ             # ECEF Z coordinate [cm]\n"
"uint32 pAcc             # Position Accuracy Estimate [cm]\n"
;
  }

  static const char* value(const ::ublox_msgs::NavPOSECEF_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.iTOW);
      stream.next(m.ecefX);
      stream.next(m.ecefY);
      stream.next(m.ecefZ);
      stream.next(m.pAcc);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavPOSECEF_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::NavPOSECEF_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::NavPOSECEF_<ContainerAllocator>& v)
  {
    s << indent << "iTOW: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.iTOW);
    s << indent << "ecefX: ";
    Printer<int32_t>::stream(s, indent + "  ", v.ecefX);
    s << indent << "ecefY: ";
    Printer<int32_t>::stream(s, indent + "  ", v.ecefY);
    s << indent << "ecefZ: ";
    Printer<int32_t>::stream(s, indent + "  ", v.ecefZ);
    s << indent << "pAcc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.pAcc);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_NAVPOSECEF_H
