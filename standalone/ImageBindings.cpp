#define PY_SSIZE_T_CLEAN
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <numpy/arrayobject.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <cstring>
#include <memory>
#include <stdexcept>

/// Own one Python reference while a native operation can throw an exception.
struct PyDelete { void operator()(PyObject* value) const { Py_XDECREF(value); } };
using Owned = std::unique_ptr<PyObject, PyDelete>;

/// Copy a grayscale or float64 OpenCV matrix to an independent NumPy array.
static PyObject* array(const cv::Mat& image) {
  npy_intp dimensions[] = {image.rows, image.cols};
  PyObject* result = PyArray_SimpleNew(2, dimensions, image.depth() == CV_8U ? NPY_UINT8 : NPY_DOUBLE);
  if (!result) throw std::runtime_error("Cannot allocate output image");
  auto* data = static_cast<unsigned char*>(PyArray_DATA(reinterpret_cast<PyArrayObject*>(result)));
  for (int row = 0; row < image.rows; ++row)
    std::memcpy(data + row * image.cols * image.elemSize(), image.ptr(row), image.cols * image.elemSize());
  return result;
}

/// Translate an OpenCV/runtime error to a normal Python ValueError.
static PyObject* failure(const std::exception& error) {
  PyErr_SetString(PyExc_ValueError, error.what()); return nullptr;
}

/// Decode raw ROS Image bytes using the declared dimensions, stride, and encoding.
static PyObject* decode_raw(PyObject*, PyObject* args) {
  const char *data, *encoding; Py_ssize_t length; int width, height, step;
  if (!PyArg_ParseTuple(args, "y#iiis", &data, &length, &width, &height, &step, &encoding)) return nullptr;
  try {
    std::string enc(encoding); int channels = 1, code = -1;
    if (enc == "rgb8") { channels = 3; code = cv::COLOR_RGB2GRAY; }
    else if (enc == "bgr8") { channels = 3; code = cv::COLOR_BGR2GRAY; }
    else if (enc == "rgba8") { channels = 4; code = cv::COLOR_RGBA2GRAY; }
    else if (enc == "bgra8") { channels = 4; code = cv::COLOR_BGRA2GRAY; }
    else if (enc == "bayer_rggb8") code = cv::COLOR_BayerBG2GRAY;
    else if (enc == "bayer_bggr8") code = cv::COLOR_BayerRG2GRAY;
    else if (enc == "bayer_gbrg8") code = cv::COLOR_BayerGR2GRAY;
    else if (enc == "bayer_grbg8") code = cv::COLOR_BayerGB2GRAY;
    else if (enc != "mono8" && enc != "8UC1") throw std::runtime_error("Unsupported image encoding: " + enc);
    if (width <= 0 || height <= 0 || step < static_cast<int64_t>(width) * channels ||
        static_cast<int64_t>(height) * step != length)
      throw std::runtime_error("Invalid image dimensions, stride, or payload length");
    cv::Mat image(height, width, CV_MAKETYPE(CV_8U, channels), const_cast<char*>(data), step), gray;
    if (code >= 0) cv::cvtColor(image, gray, code); else gray = image;
    return array(gray);
  } catch (const std::exception& error) { return failure(error); }
}

/// Decode a compressed image to grayscale without a windowing dependency.
static PyObject* decode_compressed(PyObject*, PyObject* args) {
  const char* data; Py_ssize_t length;
  if (!PyArg_ParseTuple(args, "y#", &data, &length)) return nullptr;
  try {
    if (length <= 0 || length > INT_MAX) throw std::runtime_error("Invalid compressed image size");
    auto image = cv::imdecode(cv::Mat(1, length, CV_8U, const_cast<char*>(data)), cv::IMREAD_UNCHANGED);
    if (image.empty()) throw std::runtime_error("Cannot decode compressed image");
    if (image.depth() != CV_8U) throw std::runtime_error("Only 8-bit compressed images are supported");
    cv::Mat gray;
    if (image.channels() == 3) cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else if (image.channels() == 4) cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    else if (image.channels() == 1) gray = image;
    else throw std::runtime_error("Unsupported compressed image channels");
    return array(gray);
  } catch (const std::exception& error) { return failure(error); }
}

/// Load one cached lossless grayscale image for the Python IMU optimizer.
static PyObject* read_gray(PyObject*, PyObject* args) {
  const char* path;
  if (!PyArg_ParseTuple(args, "s", &path)) return nullptr;
  try {
    auto image = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) throw std::runtime_error(std::string("Cannot read image: ") + path);
    return array(image);
  } catch (const std::exception& error) { return failure(error); }
}

/// Write a grayscale array to PNG and report write failures immediately.
static PyObject* write_gray(PyObject*, PyObject* args) {
  const char* path; PyObject* value;
  if (!PyArg_ParseTuple(args, "sO", &path, &value)) return nullptr;
  Owned data(PyArray_FROM_OTF(value, NPY_UINT8, NPY_ARRAY_CARRAY_RO));
  if (!data) return nullptr;
  auto* image = reinterpret_cast<PyArrayObject*>(data.get());
  if (PyArray_NDIM(image) != 2) { PyErr_SetString(PyExc_ValueError, "Expected grayscale matrix"); return nullptr; }
  try {
    cv::Mat pixels(PyArray_DIM(image, 0), PyArray_DIM(image, 1), CV_8U, PyArray_DATA(image));
    if (!cv::imwrite(path, pixels, {cv::IMWRITE_PNG_COMPRESSION, 1}))
      throw std::runtime_error(std::string("Cannot write image: ") + path);
    Py_RETURN_NONE;
  } catch (const std::exception& error) { return failure(error); }
}

/// Rectify pinhole/radtan or equidistant stereo using the original OpenCV formulas.
static PyObject* rectify(PyObject*, PyObject* args) {
  PyObject *values[6]; int width, height, fisheye;
  if (!PyArg_ParseTuple(args, "OOOOOOiip", &values[0], &values[1], &values[2], &values[3],
      &values[4], &values[5], &width, &height, &fisheye)) return nullptr;
  Owned owned[6]; cv::Mat matrix[6];
  for (int i = 0; i < 6; ++i) {
    owned[i].reset(PyArray_FROM_OTF(values[i], NPY_DOUBLE, NPY_ARRAY_CARRAY_RO));
    if (!owned[i]) return nullptr;
    auto* value = reinterpret_cast<PyArrayObject*>(owned[i].get());
    if (PyArray_NDIM(value) < 1 || PyArray_NDIM(value) > 2) {
      PyErr_SetString(PyExc_ValueError, "Expected vector or matrix"); return nullptr;
    }
    matrix[i] = cv::Mat(PyArray_DIM(value, 0), PyArray_NDIM(value) == 2 ? PyArray_DIM(value, 1) : 1,
      CV_64F, PyArray_DATA(value));
  }
  try {
    cv::Mat r0, r1, p0, p1, q; cv::Size size(width, height);
    if (fisheye) cv::fisheye::stereoRectify(matrix[0], matrix[1], matrix[2], matrix[3], size,
      matrix[4], matrix[5], r0, r1, p0, p1, q, cv::CALIB_ZERO_DISPARITY, size, 0.0, 1.0);
    else cv::stereoRectify(matrix[0], matrix[1], matrix[2], matrix[3], size,
      matrix[4], matrix[5], r0, r1, p0, p1, q, cv::CALIB_ZERO_DISPARITY, 0, size);
    Owned result(PyTuple_New(5)); if (!result) return nullptr;
    cv::Mat outputs[] = {r0, r1, p0, p1, q};
    for (int i = 0; i < 5; ++i) PyTuple_SET_ITEM(result.get(), i, array(outputs[i]));
    return result.release();
  } catch (const std::exception& error) { return failure(error); }
}

static PyMethodDef methods[] = {
  {"decode_raw", decode_raw, METH_VARARGS, "Decode native raw pixels with stride."},
  {"decode_compressed", decode_compressed, METH_VARARGS, "Decode PNG/JPEG to grayscale."},
  {"read_gray", read_gray, METH_VARARGS, "Read a cached grayscale frame."},
  {"write_gray", write_gray, METH_VARARGS, "Write a lossless cached frame."},
  {"rectify", rectify, METH_VARARGS, "Compute stereo rectification matrices."},
  {nullptr, nullptr, 0, nullptr}
};
static PyModuleDef module = {PyModuleDef_HEAD_INIT, "_kalibr_image", "Headless image operations.", -1, methods};
/// Initialize the small headless image module and NumPy's C API.
PyMODINIT_FUNC PyInit__kalibr_image() { import_array(); return PyModule_Create(&module); }
