#include <Exceptions.hpp>
#include <exception>

const char* PointMatchingException::what() const throw() {
    return "Exception occurred in PointMatching";
}

PointMatchingException PointMatchingEx;
