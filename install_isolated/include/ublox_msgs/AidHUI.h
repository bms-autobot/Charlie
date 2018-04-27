// Generated by gencpp from file ublox_msgs/AidHUI.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_AIDHUI_H
#define UBLOX_MSGS_MESSAGE_AIDHUI_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ublox_msgs
{
template <class ContainerAllocator>
struct AidHUI_
{
  typedef AidHUI_<ContainerAllocator> Type;

  AidHUI_()
    : health(0)
    , utcA0(0.0)
    , utcA1(0.0)
    , utcTOW(0)
    , utcWNT(0)
    , utcLS(0)
    , utcWNF(0)
    , utcDN(0)
    , utcLSF(0)
    , utcSpare(0)
    , klobA0(0.0)
    , klobA1(0.0)
    , klobA2(0.0)
    , klobA3(0.0)
    , klobB0(0.0)
    , klobB1(0.0)
    , klobB2(0.0)
    , klobB3(0.0)
    , flags(0)  {
    }
  AidHUI_(const ContainerAllocator& _alloc)
    : health(0)
    , utcA0(0.0)
    , utcA1(0.0)
    , utcTOW(0)
    , utcWNT(0)
    , utcLS(0)
    , utcWNF(0)
    , utcDN(0)
    , utcLSF(0)
    , utcSpare(0)
    , klobA0(0.0)
    , klobA1(0.0)
    , klobA2(0.0)
    , klobA3(0.0)
    , klobB0(0.0)
    , klobB1(0.0)
    , klobB2(0.0)
    , klobB3(0.0)
    , flags(0)  {
  (void)_alloc;
    }



   typedef uint32_t _health_type;
  _health_type health;

   typedef double _utcA0_type;
  _utcA0_type utcA0;

   typedef double _utcA1_type;
  _utcA1_type utcA1;

   typedef int32_t _utcTOW_type;
  _utcTOW_type utcTOW;

   typedef int16_t _utcWNT_type;
  _utcWNT_type utcWNT;

   typedef int16_t _utcLS_type;
  _utcLS_type utcLS;

   typedef int16_t _utcWNF_type;
  _utcWNF_type utcWNF;

   typedef int16_t _utcDN_type;
  _utcDN_type utcDN;

   typedef int16_t _utcLSF_type;
  _utcLSF_type utcLSF;

   typedef int16_t _utcSpare_type;
  _utcSpare_type utcSpare;

   typedef float _klobA0_type;
  _klobA0_type klobA0;

   typedef float _klobA1_type;
  _klobA1_type klobA1;

   typedef float _klobA2_type;
  _klobA2_type klobA2;

   typedef float _klobA3_type;
  _klobA3_type klobA3;

   typedef float _klobB0_type;
  _klobB0_type klobB0;

   typedef float _klobB1_type;
  _klobB1_type klobB1;

   typedef float _klobB2_type;
  _klobB2_type klobB2;

   typedef float _klobB3_type;
  _klobB3_type klobB3;

   typedef uint32_t _flags_type;
  _flags_type flags;


    enum { CLASS_ID = 11u };
     enum { MESSAGE_ID = 2u };
     enum { FLAGS_HEALTH = 1u };
     enum { FLAGS_UTC = 2u };
     enum { FLAGS_KLOB = 4u };
 

  typedef boost::shared_ptr< ::ublox_msgs::AidHUI_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::AidHUI_<ContainerAllocator> const> ConstPtr;

}; // struct AidHUI_

typedef ::ublox_msgs::AidHUI_<std::allocator<void> > AidHUI;

typedef boost::shared_ptr< ::ublox_msgs::AidHUI > AidHUIPtr;
typedef boost::shared_ptr< ::ublox_msgs::AidHUI const> AidHUIConstPtr;

// constants requiring out of line definition

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::AidHUI_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::AidHUI_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'ublox_msgs': ['/home/ubuntu/catkin_ws/src/ublox/ublox_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::AidHUI_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::AidHUI_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::AidHUI_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::AidHUI_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::AidHUI_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::AidHUI_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::AidHUI_<ContainerAllocator> >
{
  static const char* value()
  {
    return "60cd4ce940333cb9b38edd447085ce5c";
  }

  static const char* value(const ::ublox_msgs::AidHUI_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x60cd4ce940333cb9ULL;
  static const uint64_t static_value2 = 0xb38edd447085ce5cULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::AidHUI_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/AidHUI";
  }

  static const char* value(const ::ublox_msgs::AidHUI_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::AidHUI_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# AID-HUI (0x0B 0x02)\n\
# GPS Health, UTC and ionosphere parameters\n\
#\n\
# This message contains a health bit mask, UTC time and Klobuchar parameters. For more\n\
# information on these parameters, please see the ICD-GPS-200 documentation.\n\
\n\
uint8 CLASS_ID = 11\n\
uint8 MESSAGE_ID = 2\n\
\n\
uint32  health          # Bitmask, every bit represenst a GPS SV (1-32). If the bit is set the SV is healthy.\n\
float64 utcA0           # UTC - parameter A0\n\
float64 utcA1           # UTC - parameter A1\n\
int32   utcTOW          # UTC - reference time of week\n\
int16   utcWNT          # UTC - reference week number\n\
int16   utcLS           # UTC - time difference due to leap seconds before event\n\
int16   utcWNF          # UTC - week number when next leap second event occurs\n\
int16   utcDN           # UTC - day of week when next leap second event occurs\n\
int16   utcLSF          # UTC - time difference due to leap seconds after event\n\
int16   utcSpare        # UTC - Spare to ensure structure is a multiple of 4 bytes\n\
float32 klobA0          #K lobuchar - alpha 0 [s]\n\
float32 klobA1          # Klobuchar - alpha 1 [s/semicircle]\n\
float32 klobA2          # Klobuchar - alpha 2 [s/semicircle^2]\n\
float32 klobA3          # Klobuchar - alpha 3 [s/semicircle^3]\n\
float32 klobB0          #K lobuchar - beta 0  [s]\n\
float32 klobB1          # Klobuchar - beta 1  [s/semicircle]\n\
float32 klobB2          # Klobuchar - beta 2  [s/semicircle^2]\n\
float32 klobB3          # Klobuchar - beta 3  [s/semicircle^3]\n\
uint32 flags            # flags\n\
uint32 FLAGS_HEALTH = 1     # Healthmask field in this message is valid\n\
uint32 FLAGS_UTC = 2        # UTC parameter fields in this message are valid\n\
uint32 FLAGS_KLOB = 4       # Klobuchar parameter fields in this message are valid\n\
";
  }

  static const char* value(const ::ublox_msgs::AidHUI_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::AidHUI_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.health);
      stream.next(m.utcA0);
      stream.next(m.utcA1);
      stream.next(m.utcTOW);
      stream.next(m.utcWNT);
      stream.next(m.utcLS);
      stream.next(m.utcWNF);
      stream.next(m.utcDN);
      stream.next(m.utcLSF);
      stream.next(m.utcSpare);
      stream.next(m.klobA0);
      stream.next(m.klobA1);
      stream.next(m.klobA2);
      stream.next(m.klobA3);
      stream.next(m.klobB0);
      stream.next(m.klobB1);
      stream.next(m.klobB2);
      stream.next(m.klobB3);
      stream.next(m.flags);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AidHUI_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::AidHUI_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::AidHUI_<ContainerAllocator>& v)
  {
    s << indent << "health: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.health);
    s << indent << "utcA0: ";
    Printer<double>::stream(s, indent + "  ", v.utcA0);
    s << indent << "utcA1: ";
    Printer<double>::stream(s, indent + "  ", v.utcA1);
    s << indent << "utcTOW: ";
    Printer<int32_t>::stream(s, indent + "  ", v.utcTOW);
    s << indent << "utcWNT: ";
    Printer<int16_t>::stream(s, indent + "  ", v.utcWNT);
    s << indent << "utcLS: ";
    Printer<int16_t>::stream(s, indent + "  ", v.utcLS);
    s << indent << "utcWNF: ";
    Printer<int16_t>::stream(s, indent + "  ", v.utcWNF);
    s << indent << "utcDN: ";
    Printer<int16_t>::stream(s, indent + "  ", v.utcDN);
    s << indent << "utcLSF: ";
    Printer<int16_t>::stream(s, indent + "  ", v.utcLSF);
    s << indent << "utcSpare: ";
    Printer<int16_t>::stream(s, indent + "  ", v.utcSpare);
    s << indent << "klobA0: ";
    Printer<float>::stream(s, indent + "  ", v.klobA0);
    s << indent << "klobA1: ";
    Printer<float>::stream(s, indent + "  ", v.klobA1);
    s << indent << "klobA2: ";
    Printer<float>::stream(s, indent + "  ", v.klobA2);
    s << indent << "klobA3: ";
    Printer<float>::stream(s, indent + "  ", v.klobA3);
    s << indent << "klobB0: ";
    Printer<float>::stream(s, indent + "  ", v.klobB0);
    s << indent << "klobB1: ";
    Printer<float>::stream(s, indent + "  ", v.klobB1);
    s << indent << "klobB2: ";
    Printer<float>::stream(s, indent + "  ", v.klobB2);
    s << indent << "klobB3: ";
    Printer<float>::stream(s, indent + "  ", v.klobB3);
    s << indent << "flags: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.flags);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_AIDHUI_H