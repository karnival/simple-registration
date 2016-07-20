#ifndef EXCEPTIONS_INCLUDED
#define EXCEPTIONS_INCLUDED

#include <exception>

extern class PointMatchingException : public std::exception {
    virtual const char* what() const throw();
} PointMatchingEx;
#endif
