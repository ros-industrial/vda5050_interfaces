#ifndef TEST__GENERATOR__JSON_GENERATOR_HPP_
#define TEST__GENERATOR__JSON_GENERATOR_HPP_

#include <nlohmann/json.hpp>

#include "generator.hpp"

/// \brief Utility class to generate random VDA 5050 JSON objects 
class RandomJSONgenerator
{
public:

    /// \brief Generate a fully populated JSON object of a supported type 
    nlohmann::json generate()
    {

    }

private:
    enum class JsonTypes {
        Connection,
        Order,
        
    }

}

#endif