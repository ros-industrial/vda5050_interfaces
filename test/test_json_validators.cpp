#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/validators.hpp"
#include "vda5050_msgs/json_utils/schemas.hpp"
#include "generator/jsonGenerator.hpp"

/// \brief Fixture class to create VDA5050 JSON objects for tests  
class JsonValidatorTest : public ::testing::Test
{
protected:
    JsonValidatorTest() 
    {   
        connection_object_json = json_generator.generate(RandomJSONgenerator::JsonTypes::Connection);
        connection_schema_json = nlohmann::json::parse(connection_schema);
    }

    RandomJSONgenerator json_generator;
    nlohmann::json connection_object_json;
    nlohmann::json connection_schema_json;
    /// TODO: declare other VDA5050 JSON objects here
};

/// \brief tests that a valid VDA5050 JSON object passes against the schema validator
TEST_F(JsonValidatorTest, BasicValidationTest)
{
    int connection_result = validate_schema(connection_schema_json, connection_object_json);
    EXPECT_EQ(connection_result, 1);
}