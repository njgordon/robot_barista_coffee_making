// Generated by gencpp from file manipulation/ReturnJointStates.msg
// DO NOT EDIT!


#ifndef MANIPULATION_MESSAGE_RETURNJOINTSTATES_H
#define MANIPULATION_MESSAGE_RETURNJOINTSTATES_H

#include <ros/service_traits.h>


#include <manipulation/ReturnJointStatesRequest.h>
#include <manipulation/ReturnJointStatesResponse.h>


namespace manipulation
{

struct ReturnJointStates
{

typedef ReturnJointStatesRequest Request;
typedef ReturnJointStatesResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ReturnJointStates
} // namespace manipulation


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::manipulation::ReturnJointStates > {
  static const char* value()
  {
    return "ce9bd2b56c904b190a782a08482fb4e9";
  }

  static const char* value(const ::manipulation::ReturnJointStates&) { return value(); }
};

template<>
struct DataType< ::manipulation::ReturnJointStates > {
  static const char* value()
  {
    return "manipulation/ReturnJointStates";
  }

  static const char* value(const ::manipulation::ReturnJointStates&) { return value(); }
};


// service_traits::MD5Sum< ::manipulation::ReturnJointStatesRequest> should match
// service_traits::MD5Sum< ::manipulation::ReturnJointStates >
template<>
struct MD5Sum< ::manipulation::ReturnJointStatesRequest>
{
  static const char* value()
  {
    return MD5Sum< ::manipulation::ReturnJointStates >::value();
  }
  static const char* value(const ::manipulation::ReturnJointStatesRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::manipulation::ReturnJointStatesRequest> should match
// service_traits::DataType< ::manipulation::ReturnJointStates >
template<>
struct DataType< ::manipulation::ReturnJointStatesRequest>
{
  static const char* value()
  {
    return DataType< ::manipulation::ReturnJointStates >::value();
  }
  static const char* value(const ::manipulation::ReturnJointStatesRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::manipulation::ReturnJointStatesResponse> should match
// service_traits::MD5Sum< ::manipulation::ReturnJointStates >
template<>
struct MD5Sum< ::manipulation::ReturnJointStatesResponse>
{
  static const char* value()
  {
    return MD5Sum< ::manipulation::ReturnJointStates >::value();
  }
  static const char* value(const ::manipulation::ReturnJointStatesResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::manipulation::ReturnJointStatesResponse> should match
// service_traits::DataType< ::manipulation::ReturnJointStates >
template<>
struct DataType< ::manipulation::ReturnJointStatesResponse>
{
  static const char* value()
  {
    return DataType< ::manipulation::ReturnJointStates >::value();
  }
  static const char* value(const ::manipulation::ReturnJointStatesResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MANIPULATION_MESSAGE_RETURNJOINTSTATES_H