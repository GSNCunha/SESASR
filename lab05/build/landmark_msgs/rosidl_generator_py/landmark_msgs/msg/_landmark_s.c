// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from landmark_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "landmark_msgs/msg/detail/landmark__struct.h"
#include "landmark_msgs/msg/detail/landmark__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool landmark_msgs__msg__landmark__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[37];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("landmark_msgs.msg._landmark.Landmark", full_classname_dest, 36) == 0);
  }
  landmark_msgs__msg__Landmark * ros_message = _ros_message;
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // hamming
    PyObject * field = PyObject_GetAttrString(_pymsg, "hamming");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->hamming = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // goodness
    PyObject * field = PyObject_GetAttrString(_pymsg, "goodness");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->goodness = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // decision_margin
    PyObject * field = PyObject_GetAttrString(_pymsg, "decision_margin");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->decision_margin = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // range
    PyObject * field = PyObject_GetAttrString(_pymsg, "range");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->range = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bearing
    PyObject * field = PyObject_GetAttrString(_pymsg, "bearing");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bearing = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * landmark_msgs__msg__landmark__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Landmark */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("landmark_msgs.msg._landmark");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Landmark");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  landmark_msgs__msg__Landmark * ros_message = (landmark_msgs__msg__Landmark *)raw_ros_message;
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // hamming
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->hamming);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hamming", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // goodness
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->goodness);
    {
      int rc = PyObject_SetAttrString(_pymessage, "goodness", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // decision_margin
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->decision_margin);
    {
      int rc = PyObject_SetAttrString(_pymessage, "decision_margin", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // range
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->range);
    {
      int rc = PyObject_SetAttrString(_pymessage, "range", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bearing
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bearing);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bearing", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
