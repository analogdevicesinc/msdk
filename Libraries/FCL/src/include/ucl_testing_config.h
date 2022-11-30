//this file is used to define which algorithms will be tested
//this is a multi-platform file
//the platform is automatically managed by the makefile of the project

// ** DO NOT MODIFY THE FOLLOWING LINES
#include <ucl/ucl_types.h>

#ifndef _UCL_TESTING_CONFIG_H
#define _UCL_TESTING_CONFIG_H
#include <stdio.h>
#define PRINTF printf

#ifdef SYMMETRIC
#define AES
#endif
#ifdef ASYMMETRIC
#define ECDSA
#endif


int ucl_testing(void);
#endif//UCL_TESTING_CONFIG_H
