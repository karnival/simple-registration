class PointMatchingException : public std::exception {
    virtual const char* what() const throw() {
        return "Exception occurred in PointMatching.";
    }
} PointMatchingEx;

