#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <iostream>

#include "generator/jsonGenerator.hpp"
#include "vda5050_msgs/json_utils/schemas.hpp"
#include "vda5050_msgs/json_utils/validators.hpp"

/// \brief Fixture class to create VDA5050 JSON objects for tests
class JsonValidatorTest : public ::testing::Test
{
protected:
  JsonValidatorTest()
  {
    connection_object =
      json_generator.generate(RandomJSONgenerator::JsonTypes::Connection);
    connection_schema = nlohmann::json::parse(connection_schema);
    /// TODO: Instantiate other VDA5050 JSON objects
  }

  RandomJSONgenerator json_generator;
  nlohmann::json connection_object;
  nlohmann::json connection_schema;
  /// TODO: Declare other VDA5050 JSON objects
};

/// \brief Tests the is_valid_schema function to check that it passes when validating a correctly formatted JSON object
TEST_F(JsonValidatorTest, BasicValidationTest)
{
  /// TODO (@shawnkchan) Change this to a typed test so that we can iterate over the different VDA5050 object types
  std::cout << "running test" << "\n";
  int connection_result =
    is_valid_schema(connection_schema, connection_object);
  EXPECT_EQ(connection_result, true);
  std::cout << "test ended" << "\n";
}
