// Minimal declarations used only to compile-check the gateway without MATLAB.
#pragma once

#include <cstddef>

using mwIndex = std::size_t;
using mwSize = std::size_t;

struct mxArray;
enum mxComplexity { mxREAL = 0, mxCOMPLEX = 1 };
extern "C" {
bool mxIsChar(const mxArray * value);
bool mxIsClass(const mxArray * value, const char * class_name);
bool mxIsDouble(const mxArray * value);
bool mxIsComplex(const mxArray * value);
bool mxIsStruct(const mxArray * value);
mwSize mxGetNumberOfElements(const mxArray * value);
double mxGetScalar(const mxArray * value);
double * mxGetPr(const mxArray * value);
mxArray * mxGetField(const mxArray * structure, mwIndex index, const char * field_name);
char * mxArrayToString(const mxArray * value);
void mxDestroyArray(mxArray * value);
void mxFree(void * memory);

mxArray * mxCreateCellMatrix(mwSize rows, mwSize columns);
mxArray * mxCreateDoubleMatrix(mwSize rows, mwSize columns, mxComplexity complexity);
mxArray * mxCreateDoubleScalar(double value);
mxArray * mxCreateString(const char * value);
mxArray * mxCreateStructMatrix(
  mwSize rows, mwSize columns, int number_of_fields, const char ** field_names);
void mxSetField(mxArray * structure, mwIndex index, const char * field_name, mxArray * value);
void mxSetCell(mxArray * cell, mwIndex index, mxArray * value);

int mexAtExit(void (*exit_function)());
int mexCallMATLAB(
  int output_count, mxArray * outputs[], int input_count, mxArray * inputs[], const char * name);
void mexErrMsgIdAndTxt(const char * identifier, const char * format, ...);
}
