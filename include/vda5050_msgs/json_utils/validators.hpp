#include <nlohmann/json-schema.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include "include/vda5050_msgs/json_utils/schemas.hpp"

/// @brief Checks that a JSON object is following the specified schema
///
/// @param schema The schema to validate against, as an nlohmann::json object
/// @param j Reference to the nlohmann::json object to be validated
/// @return EXIT_FAILURE or EXIT_SUCCESS
int validate_schema(nlohmann::json schema, nlohmann::json& j)
{
    nlohmann::json_schema::json_validator validator;

    try
    {
        validator.set_root_schema(schema);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Validation of schema failed: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    try
    {
        validator.validate(j);
    }   
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return EXIT_SUCCESS;
}
