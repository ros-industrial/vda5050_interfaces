#ifndef TEST__GENERATOR__JSON_GENERATOR_HPP_
#define TEST__GENERATOR__JSON_GENERATOR_HPP_

#include <nlohmann/json.hpp>

#include "generator.hpp"

/// \brief Utility class to generate random VDA 5050 JSON objects 
class RandomJSONgenerator
{
public:

    /// \brief Enum values for each VDA5050 JSON object
    enum class JsonTypes {
        Connection,
        Order,
        InstantActions
    };

    /// \brief Generate a fully populated JSON object of a supported type 
    nlohmann::json generate(const JsonTypes type)
    {
        nlohmann::json j;
        RandomDataGenerator generator;

        j["headerId"] = generator.generate_uint();
        j["timestamp"] = generator.generate_random_string();
        j["version"] = generator.generate_random_string();
        j["manufacturer"] = generator.generate_random_string();
        j["serialNumber"] = generator.generate_random_string();        

        switch (type)
        {
            case JsonTypes::Connection:
                /// create Connection JSON object
                j["connectionState"] = generator.generate_connection_state();
                break;

            case JsonTypes::Order:
                /// TODO: (@shawnkchan) complete this once random generator for Order message is completed
                /// create Order JSON Object 
                break;

            case JsonTypes::InstantActions:
                /// TODO: (@shawnkchan) complete this once random generator for InstantActions message is completed
                /// create InstantActions JSON Object
                break;
        }
        return j;
    }

};

#endif