#ifndef VDA5050_MSGS__JSON_UTILS__VALIDATORS_HPP_
#define VDA5050_MSGS__JSON_UTILS__VALIDATORS_HPP_  /// TODO: change header guard name when we separate this from the VDA5050 Messages package

#include <iostream>
#include <nlohmann/json-schema.hpp>
#include <nlohmann/json.hpp>
#include <string>

#include "vda5050_msgs/json_utils/schemas.hpp"

constexpr const char* ISO8601_FORMAT = "%Y-%m-%dT%H:%M:%S";

/// \brief Utility function to check that a given string is in ISO8601 format
/// \param value The string to be checked
/// \return True if the given string follows the format
bool is_in_ISO8601_format(const std::string& value)
{
  std::tm t = {};
  char sep;
  int millisec = 0;

  std::istringstream ss(value);

  ss >> std::get_time(&t, ISO8601_FORMAT);
  if (ss.fail())
  {
    return false;
  }

  ss >> sep;
  if (ss.fail() || sep != '.')
  {
    return false;
  }

  ss >> millisec;
  if (ss.fail())
  {
    return false;
  }
  if (!ss.eof())
  {
    ss.ignore(std::numeric_limits<std::streamsize>::max(), 'Z');
  }
  else
  {
    return false;
  }
  return true;
}

/// TODO (@shawnkchan) This can probably be generalised for any other custom formats that we may need. Keeping it specific for now.
/// \brief Format checker for a date-time field
/// \param format Name of the field whose format is to be checked
/// \param value Value associated with the given field
static void date_time_format_checker(
  const std::string& format, const std::string& value)
{
  if (format == "date-time")
  {
    if (!is_in_ISO8601_format(value))
    {
      throw std::invalid_argument("Value is not in valid ISO8601 format");
    }
  }
  else
  {
    throw std::logic_error("Don't know how to validate " + format);
  }
}

/// \brief Checks that a JSON object is following the a given schema
///
/// \param schema The schema to validate against, as an nlohmann::json object
/// \param j Reference to the nlohmann::json object to be validated
/// \return true if schema is valid
bool is_valid_schema(nlohmann::json schema, nlohmann::json& j)
{
  nlohmann::json_schema::json_validator validator(
    nullptr, date_time_format_checker);

  try
  {
    validator.set_root_schema(schema);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Validation of schema failed: " << e.what() << "\n";
    return false;
  }

  try
  {
    validator.validate(j);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return false;
  }
  return true;
}

#endif