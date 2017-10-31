#include<boost/python.hpp>
#include"PWMPython.hpp"

using namespace boost::python;

BOOST_PYTHON_MODULE(pylibpwm)
{
    class_<PWMPython>("PWMPython",init(int),init(int),init(int))
        .def("setPWM", &PWMPython::setPWM)
        .def("setAllPWM",&PWMPython::setAllPwm);
}