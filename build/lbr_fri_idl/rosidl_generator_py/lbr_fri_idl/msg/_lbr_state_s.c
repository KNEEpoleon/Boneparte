// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from lbr_fri_idl:msg/LBRState.idl
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
#include "lbr_fri_idl/msg/detail/lbr_state__struct.h"
#include "lbr_fri_idl/msg/detail/lbr_state__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool lbr_fri_idl__msg__lbr_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[36];
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
    assert(strncmp("lbr_fri_idl.msg._lbr_state.LBRState", full_classname_dest, 35) == 0);
  }
  lbr_fri_idl__msg__LBRState * ros_message = _ros_message;
  {  // sample_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "sample_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->sample_time = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // session_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "session_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->session_state = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // connection_quality
    PyObject * field = PyObject_GetAttrString(_pymsg, "connection_quality");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->connection_quality = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // safety_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "safety_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->safety_state = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // operation_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "operation_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->operation_mode = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // drive_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "drive_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->drive_state = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // client_command_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "client_command_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->client_command_mode = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // overlay_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "overlay_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->overlay_type = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // control_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "control_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->control_mode = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // time_stamp_sec
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_stamp_sec");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time_stamp_sec = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // time_stamp_nano_sec
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_stamp_nano_sec");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time_stamp_nano_sec = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // measured_joint_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "measured_joint_position");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 7;
      double * dest = ros_message->measured_joint_position;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // commanded_joint_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "commanded_joint_position");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 7;
      double * dest = ros_message->commanded_joint_position;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // measured_torque
    PyObject * field = PyObject_GetAttrString(_pymsg, "measured_torque");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 7;
      double * dest = ros_message->measured_torque;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // commanded_torque
    PyObject * field = PyObject_GetAttrString(_pymsg, "commanded_torque");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 7;
      double * dest = ros_message->commanded_torque;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // external_torque
    PyObject * field = PyObject_GetAttrString(_pymsg, "external_torque");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 7;
      double * dest = ros_message->external_torque;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // ipo_joint_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "ipo_joint_position");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 7;
      double * dest = ros_message->ipo_joint_position;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // tracking_performance
    PyObject * field = PyObject_GetAttrString(_pymsg, "tracking_performance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tracking_performance = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * lbr_fri_idl__msg__lbr_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LBRState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("lbr_fri_idl.msg._lbr_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LBRState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  lbr_fri_idl__msg__LBRState * ros_message = (lbr_fri_idl__msg__LBRState *)raw_ros_message;
  {  // sample_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->sample_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sample_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // session_state
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->session_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "session_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // connection_quality
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->connection_quality);
    {
      int rc = PyObject_SetAttrString(_pymessage, "connection_quality", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // safety_state
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->safety_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "safety_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // operation_mode
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->operation_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "operation_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // drive_state
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->drive_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "drive_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // client_command_mode
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->client_command_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "client_command_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // overlay_type
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->overlay_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "overlay_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // control_mode
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->control_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "control_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_stamp_sec
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->time_stamp_sec);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_stamp_sec", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_stamp_nano_sec
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->time_stamp_nano_sec);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_stamp_nano_sec", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // measured_joint_position
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "measured_joint_position");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->measured_joint_position[0]);
    memcpy(dst, src, 7 * sizeof(double));
    Py_DECREF(field);
  }
  {  // commanded_joint_position
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "commanded_joint_position");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->commanded_joint_position[0]);
    memcpy(dst, src, 7 * sizeof(double));
    Py_DECREF(field);
  }
  {  // measured_torque
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "measured_torque");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->measured_torque[0]);
    memcpy(dst, src, 7 * sizeof(double));
    Py_DECREF(field);
  }
  {  // commanded_torque
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "commanded_torque");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->commanded_torque[0]);
    memcpy(dst, src, 7 * sizeof(double));
    Py_DECREF(field);
  }
  {  // external_torque
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "external_torque");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->external_torque[0]);
    memcpy(dst, src, 7 * sizeof(double));
    Py_DECREF(field);
  }
  {  // ipo_joint_position
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "ipo_joint_position");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->ipo_joint_position[0]);
    memcpy(dst, src, 7 * sizeof(double));
    Py_DECREF(field);
  }
  {  // tracking_performance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tracking_performance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tracking_performance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
